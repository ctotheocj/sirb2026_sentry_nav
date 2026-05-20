// Copyright 2026 Lihan Chen
//
// Licensed under the Apache License, Version 2.0

#include "global_relocalization/scan_context.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace global_relocalization
{

ScanContextDescriptor::ScanContextDescriptor(int rings, int sectors, double max_range)
{
  reset(rings, sectors, max_range);
}

void ScanContextDescriptor::reset(int rings, int sectors, double max_range)
{
  rings_ = std::max(1, rings);
  sectors_ = std::max(1, sectors);
  max_range_ = std::max(0.1, max_range);
  data_.assign(static_cast<size_t>(rings_ * sectors_), 0.0f);
}

bool ScanContextDescriptor::build(const pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  if (rings_ <= 0 || sectors_ <= 0 || max_range_ <= 0.0) {
    return false;
  }
  std::fill(data_.begin(), data_.end(), 0.0f);
  size_t used = 0;
  for (const auto & p : cloud.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    const double range = std::hypot(static_cast<double>(p.x), static_cast<double>(p.y));
    if (range <= 1.0e-3 || range > max_range_) {
      continue;
    }
    const int ring = std::min(
      rings_ - 1, static_cast<int>(std::floor(range / max_range_ * static_cast<double>(rings_))));
    double angle = std::atan2(static_cast<double>(p.y), static_cast<double>(p.x));
    if (angle < 0.0) {
      angle += 2.0 * M_PI;
    }
    const int sector = std::min(
      sectors_ - 1,
      static_cast<int>(std::floor(angle / (2.0 * M_PI) * static_cast<double>(sectors_))));
    const size_t idx = static_cast<size_t>(ring * sectors_ + sector);
    const float value = static_cast<float>(std::max(0.0, static_cast<double>(p.z) + 2.0));
    data_[idx] = std::max(data_[idx], value);
    used++;
  }
  return used > 0;
}

double ScanContextDescriptor::similarityTo(
  const ScanContextDescriptor & other, int * best_shift) const
{
  if (
    rings_ != other.rings_ || sectors_ != other.sectors_ || data_.empty() || other.data_.empty())
  {
    if (best_shift) {
      *best_shift = 0;
    }
    return 0.0;
  }

  double best = -std::numeric_limits<double>::infinity();
  int best_s = 0;
  for (int shift = 0; shift < sectors_; ++shift) {
    const double score = cosineForShift(other, shift);
    if (score > best) {
      best = score;
      best_s = shift;
    }
  }
  if (best_shift) {
    *best_shift = best_s;
  }
  return std::max(0.0, best);
}

double ScanContextDescriptor::cosineForShift(
  const ScanContextDescriptor & other, int shift) const
{
  double dot = 0.0;
  double norm_a = 0.0;
  double norm_b = 0.0;
  for (int r = 0; r < rings_; ++r) {
    for (int s = 0; s < sectors_; ++s) {
      const size_t a_idx = static_cast<size_t>(r * sectors_ + s);
      const int shifted = (s + shift) % sectors_;
      const size_t b_idx = static_cast<size_t>(r * sectors_ + shifted);
      const double a = data_[a_idx];
      const double b = other.data_[b_idx];
      dot += a * b;
      norm_a += a * a;
      norm_b += b * b;
    }
  }
  if (norm_a <= 1.0e-9 || norm_b <= 1.0e-9) {
    return 0.0;
  }
  return dot / (std::sqrt(norm_a) * std::sqrt(norm_b));
}

}  // namespace global_relocalization
