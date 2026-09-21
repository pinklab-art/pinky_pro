#!/usr/bin/env bash
# Stop every pinky_mujoco simulator process (bridge, launch, robot_state_publisher) left running.
# Excludes shells whose command line merely mentions these names.
pids=$(ps -eo pid=,args= | grep -E 'pinky_mujoco/bridge|launch_sim\.launch\.py|robot_state_publisher/robot_state_publisher' \
       | grep -v -E 'grep|bash -c|stop_sim' | awk '{print $1}')
if [ -n "$pids" ]; then
  kill -INT $pids 2>/dev/null
  sleep 2
  kill -KILL $pids 2>/dev/null
fi
left=$(ps -eo pid=,args= | grep -E 'pinky_mujoco/bridge' | grep -v -E 'grep|bash -c|stop_sim' | wc -l)
echo "pinky_mujoco bridges running: $left"
