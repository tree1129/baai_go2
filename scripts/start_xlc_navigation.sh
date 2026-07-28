#!/usr/bin/env bash
# XLC startup: MID360 -> FAST-LIO -> Open3D localization -> navigation -> RViz.
# Motion is deliberately armed in a separate, explicit step.
# Run as: /home/unitree/start_xlc_navigation.sh start
# Do NOT run this whole script with sudo. It requests sudo only for the LiDAR NIC address.
set -eo pipefail

ROS_SETUP=/opt/ros/noetic/setup.bash
LIVOX_WS=/home/unitree/zzx/loca_ws
LOC_WS=/home/unitree/FAST_LIO_LOCALIZATION_HUMANOID
NAV_WS=/home/unitree/go2_navigation250917251016
SDK=/home/unitree/go_ros_sdk/gogo/unitree_sdk2
RVIZ_CONFIG=/home/unitree/xlc_navigation.rviz
LOG_DIR=/tmp/xlc_navigation
LIDAR_IP=192.168.1.181
LIDAR_HOST_IP=192.168.1.50
CONTROL_INTERFACE=go2ctrl

source "$ROS_SETUP"
mkdir -p "$LOG_DIR"

node_exists() { rosnode list 2>/dev/null | grep -qx "$1"; }
rviz_running() {
  local rviz_node
  while read -r rviz_node; do
    [[ -n "$rviz_node" ]] &&
      rosnode ping -c 1 "$rviz_node" >/dev/null 2>&1 &&
      return 0
  done < <(rosnode list 2>/dev/null | grep '^/rviz' || true)
  return 1
}
start_background() {
  local log_file=$1
  shift
  nohup setsid "$@" >"$LOG_DIR/$log_file" 2>&1 </dev/null &
}
wait_for_topic() {
  local topic=$1 seconds=${2:-30}
  timeout "$seconds" rostopic echo -n 1 "$topic" >/dev/null 2>&1
}
fail_with_log() {
  local description=$1 logfile=$2
  echo "错误：${description}" >&2
  [[ -f "$LOG_DIR/$logfile" ]] && tail -30 "$LOG_DIR/$logfile" >&2 || true
  exit 1
}

find_lidar_interface() {
  # Prefer an interface already holding the Livox host address; it remains correct after reboot/replug.
  local candidate
  for candidate in eth0 eth1; do
    ip -4 addr show dev "$candidate" 2>/dev/null | grep -q "${LIDAR_HOST_IP}/" && {
      printf '%s\n' "$candidate"; return 0;
    }
  done
  # Otherwise use the only wired interface that has carrier.
  local found=""
  for candidate in eth0 eth1; do
    [[ -r "/sys/class/net/$candidate/carrier" ]] && [[ "$(cat "/sys/class/net/$candidate/carrier")" == 1 ]] || continue
    if [[ -n "$found" ]]; then
      echo "检测到多个有线网口，请先将雷达接入唯一网口，或为雷达口配置 ${LIDAR_HOST_IP}。" >&2
      return 1
    fi
    found=$candidate
  done
  [[ -n "$found" ]] && printf '%s\n' "$found"
}

prepare_lidar_network() {
  LIDAR_INTERFACE="$(find_lidar_interface)" || {
    echo "错误：未找到雷达网口。请检查 MID360 网线和供电。" >&2; exit 1;
  }
  echo "雷达网口：$LIDAR_INTERFACE；配置 ${LIDAR_HOST_IP}/24。"
  # Reuse an existing sudo ticket in non-interactive launch sessions; otherwise prompt once.
  sudo -n true 2>/dev/null || sudo -v
  # Livox driver must use .1.50 as the preferred source address. Reorder addresses after a reboot.
  sudo ip addr del 192.168.123.18/24 dev "$LIDAR_INTERFACE" 2>/dev/null || true
  sudo ip addr del "${LIDAR_HOST_IP}/24" dev "$LIDAR_INTERFACE" 2>/dev/null || true
  sudo ip addr add "${LIDAR_HOST_IP}/24" dev "$LIDAR_INTERFACE"
  # Livox and the Go2 body controller share the cable but require different
  # preferred source addresses. A macvlan gives DDS its own .123.18 interface.
  sudo ip link del "$CONTROL_INTERFACE" 2>/dev/null || true
  sudo ip link add link "$LIDAR_INTERFACE" name "$CONTROL_INTERFACE" type macvlan mode bridge
  sudo ip addr add 192.168.123.18/24 dev "$CONTROL_INTERFACE"
  sudo ip link set "$CONTROL_INTERFACE" up
  ping -I "$CONTROL_INTERFACE" -c 1 -W 1 192.168.123.161 >/dev/null ||
    fail_with_log "机身控制器 192.168.123.161 不可达。" safe_bridge.log
}

start_stack() {
  # A normal start is always safe/disarmed, even if a bridge survived from a
  # previous interactive enable-motion session.
  timeout 2 rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID '{}' >/dev/null 2>&1 || true
  node_exists /unitree_safe_cmd_vel_bridge &&
    rosnode kill /unitree_safe_cmd_vel_bridge >/dev/null 2>&1 || true
  prepare_lidar_network
  if ! rostopic list >/dev/null 2>&1; then
    start_background roscore.log roscore
    sleep 3
  fi

  source "$LIVOX_WS/devel/setup.bash"
  if ! node_exists /livox_lidar_publisher2; then
    start_background livox.log roslaunch livox_ros_driver2 msg_MID360.launch
  fi
  wait_for_topic /livox/lidar 25 || fail_with_log "25 秒内没有收到 /livox/lidar；请检查 ${LIDAR_INTERFACE}、${LIDAR_IP} 和雷达供电。" livox.log

  source "$LOC_WS/devel/setup.bash"
  node_exists /fast_lio_node || start_background fast_lio.log roslaunch fast_lio mapping_mid360_g1.launch
  node_exists /global_localization_node || start_background localization.log roslaunch open3d_loc open3d_loc_g1.launch
  echo "定位模块启动中..."
  wait_for_topic /Odometry_loc 45 || fail_with_log "45 秒内没有收到 /Odometry_loc；Livox/FAST-LIO/定位图未就绪。" localization.log
  wait_for_topic /map 90 || fail_with_log "90 秒内没有收到 /map；指定的静态 PLY 地图未能发布。" localization.log

  source "$NAV_WS/devel/setup.bash"
  node_exists /go_pointcloud_transformer || start_background cloud_transform.log rosrun go_pointcloud_transformer go_pointcloud_transformer
  node_exists /xlc_odom_relay || start_background odom_relay.log rosrun topic_tools relay /Odometry_loc /go_odom __name:=xlc_odom_relay

  # Avoid a second publisher overwriting the calibrated body -> go2_link transform.
  node_exists /go2_tf_publisher && rosnode kill /go2_tf_publisher >/dev/null 2>&1 || true
  node_exists /xlc_planar_base_tf || start_background planar_tf.log /home/unitree/xlc_planar_base_tf.py
  node_exists /xlc_go2_to_laser_link || start_background laser_tf.log rosrun tf static_transform_publisher 0 0 0 0 0 0 1 go2_link laser_link 50 __name:=xlc_go2_to_laser_link
  node_exists /move_base || start_background navigation.log roslaunch robot_navigation navigationcopy2.launch
  wait_for_topic /laser_scan 25 || fail_with_log "没有 /laser_scan，导航未启动。" navigation.log
  wait_for_topic /map_grid 120 || fail_with_log "120 秒内没有 /map_grid；静态 PLY 的完整 2D 投影未生成。" navigation.log

  if ! rviz_running; then
    start_background rviz.log env DISPLAY=:1 XAUTHORITY=/home/unitree/.Xauthority rviz -d "$RVIZ_CONFIG"
  fi
  for _ in $(seq 1 10); do
    rviz_running && break
    sleep 1
  done
  if ! rviz_running; then
    # A stale RViz registration can briefly survive a restart. Retry once
    # with a fresh anonymous node name before declaring startup failure.
    start_background rviz.log env DISPLAY=:1 XAUTHORITY=/home/unitree/.Xauthority rviz -d "$RVIZ_CONFIG"
    for _ in $(seq 1 10); do
      rviz_running && break
      sleep 1
    done
  fi
  rviz_running || fail_with_log "RViz 未能在机器人桌面 DISPLAY=:1 启动。" rviz.log
  echo "启动成功：雷达、定位、规划与 RViz 已就绪。"
  echo "运动桥保持关闭；确认机器人稳定站立后执行："
  echo "  $0 enable-motion"
}

enable_motion() {
  for required_node in /livox_lidar_publisher2 /fast_lio_node /global_localization_node /move_base /xlc_planar_base_tf; do
    node_exists "$required_node" ||
      fail_with_log "缺少节点 ${required_node}；请先执行 $0 start。" navigation.log
  done
  wait_for_topic /Odometry_loc 5 ||
    fail_with_log "没有 /Odometry_loc，拒绝启用运动。" localization.log
  wait_for_topic /laser_scan 5 ||
    fail_with_log "没有 /laser_scan，拒绝启用运动。" navigation.log
  wait_for_topic /map_grid 5 ||
    fail_with_log "没有 /map_grid，拒绝启用运动。" navigation.log

  local confidence
  confidence="$(timeout 4 rostopic echo -n 1 /localization_3d_confidence 2>/dev/null |
    awk '/data:/{print $2; exit}')"
  [[ -n "$confidence" ]] ||
    fail_with_log "没有定位置信度，拒绝启用运动。" localization.log
  awk -v value="$confidence" 'BEGIN { exit !(value >= 0.30) }' ||
    fail_with_log "定位置信度 ${confidence} 低于 0.30，拒绝启用运动。" localization.log

  if [[ "${XLC_CONFIRM_STANDING:-}" != "YES" ]]; then
    if [[ -t 0 ]]; then
      echo "安全确认：机器人必须稳定站立、处于行走模式，且 Unitree App 已退出。"
      read -r -p "确认后输入 STANDING： " answer
      [[ "$answer" == "STANDING" ]] || {
        echo "未确认，运动桥未启动。"; exit 1;
      }
    else
      echo "非交互运行时请设置 XLC_CONFIRM_STANDING=YES；运动桥未启动。" >&2
      exit 1
    fi
  fi

  ip link show "$CONTROL_INTERFACE" >/dev/null 2>&1 || prepare_lidar_network
  ping -I "$CONTROL_INTERFACE" -c 1 -W 1 192.168.123.161 >/dev/null ||
    fail_with_log "机身控制器不可达，拒绝启用运动。" safe_bridge.log

  # Never arm against an old RViz goal left over from a previous run.
  rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID '{}' >/dev/null 2>&1 || true
  export LD_LIBRARY_PATH="$SDK/thirdparty/lib/aarch64:$SDK/lib/aarch64:${LD_LIBRARY_PATH:-}"
  node_exists /unitree_safe_cmd_vel_bridge ||
    start_background safe_bridge.log "$SDK/build/bin/go2_move_sub" "$CONTROL_INTERFACE"

  local attempt
  for attempt in $(seq 1 20); do
    rostopic info /cmd_vel 2>/dev/null |
      grep -q '/unitree_safe_cmd_vel_bridge' && break
    sleep 1
  done
  rostopic info /cmd_vel 2>/dev/null |
    grep -q '/unitree_safe_cmd_vel_bridge' ||
    fail_with_log "运动桥未订阅 /cmd_vel，机器人不会运动。" safe_bridge.log

  echo "运动已启用，定位置信度：${confidence}。"
  echo "现在可在 RViz 使用 2D Nav Goal 下发一个近距离、空旷区域内的新目标。"
}

disable_motion() {
  rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID '{}' >/dev/null 2>&1 || true
  if node_exists /unitree_safe_cmd_vel_bridge; then
    for _ in 1 2 3; do
      rostopic pub -1 /cmd_vel geometry_msgs/Twist '{}' >/dev/null 2>&1 || true
    done
    rosnode kill /unitree_safe_cmd_vel_bridge >/dev/null 2>&1 || true
  fi
  echo "目标已取消，运动桥已关闭。"
}

stop_stack() {
  disable_motion
  rosnode list 2>/dev/null | grep '^/rviz' | while read -r rviz_node; do
    rosnode kill "$rviz_node" >/dev/null 2>&1 || true
  done
  for _ in $(seq 1 10); do
    rosnode list 2>/dev/null | grep -q '^/rviz' || break
    sleep 1
  done
  for node in /unitree_safe_cmd_vel_bridge /move_base /octomap_projector /pointcloud_to_laserscan /go_pointcloud_transformer /xlc_odom_relay /xlc_planar_base_tf /xlc_go2_to_laser_link /map_grid_to_map /global_localization_node /fast_lio_node /livox_lidar_publisher2; do
    node_exists "$node" && rosnode kill "$node" >/dev/null 2>&1 || true
  done
  echo "已取消目标并停止雷达、定位、规划和运动桥。"
}

show_status() {
  rosnode list 2>/dev/null | grep -E 'livox|fast_lio|global_localization|move_base|safe_cmd_vel|rviz' || true
  echo "定位置信度："
  timeout 3 rostopic echo -n 1 /localization_3d_confidence 2>/dev/null || true
  echo "全局 PLY 地图："
  timeout 5 rostopic echo -n 1 /map/width 2>/dev/null || true
  echo "/cmd_vel："
  rostopic info /cmd_vel 2>/dev/null || true
}

case "${1:-start}" in
  start) start_stack ;;
  enable-motion|arm) enable_motion ;;
  disable-motion|disarm) disable_motion ;;
  stop) stop_stack ;;
  restart) stop_stack; sleep 2; start_stack ;;
  status) show_status ;;
  *) echo "用法：$0 {start|enable-motion|disable-motion|stop|restart|status}" >&2; exit 2 ;;
esac
