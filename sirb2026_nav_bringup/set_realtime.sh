#!/bin/bash
# set_realtime.sh — 提升 ROS 2 进程实时优先级 + 调整 CPU 调频策略
# 用法：sudo ./set_realtime.sh [nav_node_name]
#
# 功能：
#   1. 将指定进程（默认 component_container_mt）的调度策略改为 SCHED_FIFO, 优先级 50
#   2. 将 CPU 调频策略改为 performance（禁用节能降频）
#   3. 禁用 CPU 频率缩放的 ondemand/powersave governor
#
# 注意：需要 root 权限

set -e

PROC_NAME="${1:-component_container_mt}"

# --- 1. CPU 调频策略 → performance ---
for cpu_dir in /sys/devices/system/cpu/cpu*/cpufreq; do
    if [ -f "${cpu_dir}/scaling_governor" ]; then
        echo performance > "${cpu_dir}/scaling_governor" 2>/dev/null || true
        echo "[set_realtime] $(dirname $cpu_dir | xargs basename): governor=performance"
    fi
done

# --- 2. 提高进程实时优先级 ---
PIDS=$(pgrep -f "${PROC_NAME}" 2>/dev/null || true)

if [ -z "${PIDS}" ]; then
    echo "[set_realtime] WARNING: No process matching '${PROC_NAME}' found. Skipping RT priority."
    echo "[set_realtime] Run this script AFTER launching the nav stack."
else
    for PID in ${PIDS}; do
        chrt --fifo -p 50 "${PID}" && \
            echo "[set_realtime] PID ${PID} (${PROC_NAME}): SCHED_FIFO pri=50" || \
            echo "[set_realtime] WARNING: chrt failed for PID ${PID}"
    done
fi

# --- 3. 提高网络发送线程优先级（CycloneDDS）---
DDS_PIDS=$(pgrep -f "cyclone\|fastrtps\|rmw" 2>/dev/null || true)
for PID in ${DDS_PIDS}; do
    chrt --fifo -p 40 "${PID}" 2>/dev/null && \
        echo "[set_realtime] DDS PID ${PID}: SCHED_FIFO pri=40" || true
done

# --- 4. IRQ 平衡：将网卡中断固定到 CPU0（减少主控 CPU 中断） ---
IFACE=$(ip route | grep default | awk '{print $5}' | head -1)
if [ -n "${IFACE}" ]; then
    IRQ_NUM=$(grep "${IFACE}" /proc/interrupts 2>/dev/null | awk -F: '{print $1}' | tr -d ' ' | head -1)
    if [ -n "${IRQ_NUM}" ]; then
        echo 1 > "/proc/irq/${IRQ_NUM}/smp_affinity" 2>/dev/null && \
            echo "[set_realtime] NIC ${IFACE} IRQ${IRQ_NUM} pinned to CPU0" || true
    fi
fi

echo "[set_realtime] Done."
