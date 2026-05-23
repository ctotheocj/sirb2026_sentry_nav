#include "sirb_nav2_plugins/bt_nodes/localization_ready_condition.hpp"

#include <algorithm>
#include <cctype>
#include <limits>
#include <sstream>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"

namespace sirb_nav2_plugins
{
namespace
{

std::string trim(const std::string & input)
{
  auto begin = input.begin();
  while (begin != input.end() && std::isspace(static_cast<unsigned char>(*begin))) {
    ++begin;
  }
  auto end = input.end();
  while (end != begin && std::isspace(static_cast<unsigned char>(*(end - 1)))) {
    --end;
  }
  return std::string(begin, end);
}

}  // namespace

LocalizationReadyCondition::LocalizationReadyCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());
}

BT::PortsList LocalizationReadyCondition::providedPorts()
{
  const std::string default_topics = "ndt_omp_relocalization/diagnostics";
  return {
    BT::InputPort<std::string>(
      "diagnostics_topics", default_topics,
      "Comma-separated localization diagnostic_msgs/DiagnosticArray topics"),
    BT::InputPort<std::string>(
      "diagnostics_topic", "",
      "Deprecated single localization diagnostics topic; use diagnostics_topics"),
    BT::InputPort<double>("max_age", 0.8, "Maximum accepted diagnostics age in seconds"),
    BT::InputPort<int>("required_ready_ticks", 2, "Consecutive ready BT ticks required"),
    BT::InputPort<std::string>("required_state", "trusted", "Required localization_state value"),
    BT::InputPort<bool>("require_trusted", true, "Require diagnostics key trusted=true"),
    BT::InputPort<bool>(
      "allow_missing_diagnostics", false,
      "Return SUCCESS before diagnostics are received"),
  };
}

std::vector<std::string> LocalizationReadyCondition::getDiagnosticsTopics()
{
  const std::string default_topics = "ndt_omp_relocalization/diagnostics";
  std::string topics_text = default_topics;
  getInput("diagnostics_topics", topics_text);
  std::string legacy_topic;
  if (
    getInput("diagnostics_topic", legacy_topic) &&
    !trim(legacy_topic).empty() &&
    topics_text == default_topics)
  {
    topics_text = legacy_topic;
  }

  std::vector<std::string> topics;
  std::stringstream ss(topics_text);
  std::string item;
  while (std::getline(ss, item, ',')) {
    item = trim(item);
    if (!item.empty()) {
      topics.push_back(item);
    }
  }
  if (topics.empty()) {
    topics.push_back("ndt_omp_relocalization/diagnostics");
  }
  return topics;
}

void LocalizationReadyCondition::ensureSubscriptions(
  const std::vector<std::string> & diagnostics_topics)
{
  if (!diagnostics_subs_.empty() && diagnostics_topics_ == diagnostics_topics) {
    return;
  }

  diagnostics_topics_ = diagnostics_topics;
  diagnostics_subs_.clear();
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  diagnostics_subs_.reserve(diagnostics_topics_.size());
  for (const auto & topic : diagnostics_topics_) {
    diagnostics_subs_.push_back(
      node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        topic, rclcpp::QoS(10),
        [this, topic](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
          diagnosticsCallbackForTopic(topic, msg);
        },
        options));
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_by_topic_.clear();
    ready_count_ = 0;
    last_ready_ = false;
    has_reported_state_ = false;
    last_reported_topic_.clear();
    last_reported_state_.clear();
    last_reported_message_.clear();
    last_reported_trusted_ = false;
    last_reported_stale_ = false;
  }
}

void LocalizationReadyCondition::diagnosticsCallback(
  const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{
  diagnosticsCallbackForTopic("diagnostics", msg);
}

void LocalizationReadyCondition::diagnosticsCallbackForTopic(
  const std::string & topic,
  const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{
  for (const auto & status : msg->status) {
    if (status.name.find("relocalization") == std::string::npos &&
      status.name.find("gicp") == std::string::npos &&
      status.name.find("ndt") == std::string::npos)
    {
      continue;
    }

    std::string state = "UNKNOWN";
    bool trusted = false;
    bool has_state = false;
    bool has_trusted = false;
    for (const auto & kv : status.values) {
      if (kv.key == "localization_state") {
        state = kv.value;
        has_state = true;
      } else if (kv.key == "trusted") {
        trusted = kv.value == "true" || kv.value == "1" || kv.value == "True";
        has_trusted = true;
      }
    }

    if (!has_state) {
      continue;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    auto & snapshot = status_by_topic_[topic];
    snapshot.received_time = node_->now();
    snapshot.state = state;
    snapshot.message = status.message;
    snapshot.trusted = has_trusted ? trusted : status.level == diagnostic_msgs::msg::DiagnosticStatus::OK;
    snapshot.have_status = true;
    return;
  }
}

BT::NodeStatus LocalizationReadyCondition::onStart()
{
  return checkReady();
}

BT::NodeStatus LocalizationReadyCondition::onRunning()
{
  return checkReady();
}

void LocalizationReadyCondition::onHalted()
{
  std::lock_guard<std::mutex> lock(mutex_);
  ready_count_ = 0;
  last_ready_ = false;
  has_reported_state_ = false;
  last_reported_topic_.clear();
  last_reported_state_.clear();
  last_reported_message_.clear();
  last_reported_trusted_ = false;
  last_reported_stale_ = false;
}

BT::NodeStatus LocalizationReadyCondition::checkReady()
{
  double max_age = 0.8;
  int required_ready_ticks = 2;
  std::string required_state = "TRACKING";
  bool require_trusted = true;
  bool allow_missing_diagnostics = false;

  getInput("max_age", max_age);
  getInput("required_ready_ticks", required_ready_ticks);
  getInput("required_state", required_state);
  getInput("require_trusted", require_trusted);
  getInput("allow_missing_diagnostics", allow_missing_diagnostics);

  max_age = std::max(0.05, max_age);
  required_ready_ticks = std::max(1, required_ready_ticks);
  ensureSubscriptions(getDiagnosticsTopics());
  callback_group_executor_.spin_some();

  bool ready = false;
  double age = std::numeric_limits<double>::infinity();
  std::string state;
  std::string message;
  bool trusted = false;
  std::string selected_topic;
  bool have_any_status = false;
  bool stale = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & topic : diagnostics_topics_) {
      const auto it = status_by_topic_.find(topic);
      if (it == status_by_topic_.end() || !it->second.have_status) {
        continue;
      }
      const auto & snapshot = it->second;
      have_any_status = true;
      const double snapshot_age = (node_->now() - snapshot.received_time).seconds();
      const bool snapshot_ready =
        snapshot_age <= max_age &&
        snapshot.state == required_state &&
        (!require_trusted || snapshot.trusted);
      if (selected_topic.empty() || snapshot_ready || snapshot_age < age) {
        selected_topic = topic;
        age = snapshot_age;
        state = snapshot.state;
        message = snapshot.message;
        trusted = snapshot.trusted;
      }
      if (snapshot_ready) {
        ready = true;
        break;
      }
    }

    if (!have_any_status) {
      ready = allow_missing_diagnostics;
      state = "NO_DIAGNOSTICS";
      message = "waiting_for_localization_diagnostics";
      stale = false;
    } else {
      stale = age > max_age;
    }

    if (ready) {
      ready_count_++;
    } else {
      ready_count_ = 0;
    }
    ready = ready_count_ >= required_ready_ticks;

    const bool should_report =
      ready != last_ready_ ||
      !has_reported_state_ ||
      selected_topic != last_reported_topic_ ||
      state != last_reported_state_ ||
      message != last_reported_message_ ||
      trusted != last_reported_trusted_ ||
      stale != last_reported_stale_;
    if (should_report) {
      if (ready) {
        RCLCPP_INFO(
          node_->get_logger(),
          "LocalizationReady: ready topic='%s' state='%s' trusted=%d age=%.3fs",
          selected_topic.c_str(), state.c_str(), trusted ? 1 : 0, age);
      } else {
        RCLCPP_WARN(
          node_->get_logger(),
          "LocalizationReady: blocked topic='%s' state='%s' trusted=%d age=%.3fs message='%s'",
          selected_topic.c_str(), state.c_str(), trusted ? 1 : 0, age, message.c_str());
      }
      last_ready_ = ready;
      has_reported_state_ = true;
      last_reported_topic_ = selected_topic;
      last_reported_state_ = state;
      last_reported_message_ = message;
      last_reported_trusted_ = trusted;
      last_reported_stale_ = stale;
    }
  }

  return ready ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

}  // namespace sirb_nav2_plugins
