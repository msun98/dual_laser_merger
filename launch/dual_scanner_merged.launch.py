# Copyright 2024 pradyum
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math

from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# ===== 현장에 맞게 수정 =====
HOST_IP = "192.168.2.2"   # enp47s0 의 실제 IP

FRONT_IP = "192.168.2.10"
REAR_IP = "192.168.2.11"

# base_link 기준 라이다 장착 위치. 로봇 설정 파일의 값을 그대로 적는다.
# (x, y, z) [m], (roll, pitch, yaw) [도]
FRONT_POSE = (0.32, -0.225, 0.20, 180.0, 0.0, -45.0)
REAR_POSE = (-0.32, 0.225, 0.20, 180.0, 0.0, 135.0)

# 회전 합성 순서. 안 맞으면 "XYZ" 로 바꿔서 시험한다.
#   "ZYX" = Rz(yaw) @ Ry(pitch) @ Rx(roll)   <- ROS/tf2 기본
#   "XYZ" = Rx(roll) @ Ry(pitch) @ Rz(yaw)
ROT_ORDER = "ZYX"

TARGET_FRAME = "base_link"
# ===========================


def sick_node(name, sensor_ip, frame_id):
    # 드라이버는 전역 /scan 으로 퍼블리시한다. 두 대가 겹치므로 /<name>/scan 으로 리맵.
    return Node(
        package="sick_safetyscanners2",
        executable="sick_safetyscanners2_node",
        name=name,
        output="screen",
        emulate_tty=True,
        remappings=[
            ("scan", f"/{name}/scan"),
            ("extended_scan", f"/{name}/extended_scan"),
        ],
        parameters=[{
            "frame_id": frame_id,
            "sensor_ip": sensor_ip,
            "host_ip": HOST_IP,
            "interface_ip": "0.0.0.0",
            "host_udp_port": 0,          # 0 = OS가 포트 자동 할당 (두 대 충돌 없음)
            "channel": 0,
            "channel_enabled": True,
            "skip": 0,
            "angle_start": 0.0,
            "angle_end": 0.0,
            "time_offset": 0.0,
            "general_system_state": True,
            "derived_settings": True,
            "measurement_data": True,
            "intrusion_data": True,
            "application_io_data": True,
            "use_persistent_config": False,
            "min_intensities": 0.0,
        }],
    )


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def rot_x(t):
    c, s = math.cos(t), math.sin(t)
    return [[1, 0, 0], [0, c, -s], [0, s, c]]


def rot_y(t):
    c, s = math.cos(t), math.sin(t)
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]]


def rot_z(t):
    c, s = math.cos(t), math.sin(t)
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def mat_to_quat(m):
    """회전행렬 -> 쿼터니언 (x, y, z, w). Shepperd 방식."""
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0.0:
        k = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * k
        x = (m[2][1] - m[1][2]) / k
        y = (m[0][2] - m[2][0]) / k
        z = (m[1][0] - m[0][1]) / k
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        k = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w = (m[2][1] - m[1][2]) / k
        x = 0.25 * k
        y = (m[0][1] + m[1][0]) / k
        z = (m[0][2] + m[2][0]) / k
    elif m[1][1] > m[2][2]:
        k = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w = (m[0][2] - m[2][0]) / k
        x = (m[0][1] + m[1][0]) / k
        y = 0.25 * k
        z = (m[1][2] + m[2][1]) / k
    else:
        k = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w = (m[1][0] - m[0][1]) / k
        x = (m[0][2] + m[2][0]) / k
        y = (m[1][2] + m[2][1]) / k
        z = 0.25 * k
    return x, y, z, w


def pose_to_quat(roll_deg, pitch_deg, yaw_deg):
    rx = rot_x(math.radians(roll_deg))
    ry = rot_y(math.radians(pitch_deg))
    rz = rot_z(math.radians(yaw_deg))
    if ROT_ORDER == "ZYX":
        m = mat_mul(mat_mul(rz, ry), rx)
    else:
        m = mat_mul(mat_mul(rx, ry), rz)
    return mat_to_quat(m)


def static_tf(child, pose):
    x, y, z, roll, pitch, yaw = pose
    qx, qy, qz, qw = pose_to_quat(roll, pitch, yaw)
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"tf_{child}",
        arguments=[
            "--x", f"{x}", "--y", f"{y}", "--z", f"{z}",
            "--qx", f"{qx}", "--qy", f"{qy}", "--qz", f"{qz}", "--qw", f"{qw}",
            "--frame-id", TARGET_FRAME, "--child-frame-id", child,
        ],
    )


def generate_launch_description():
    front = sick_node("front_scanner", FRONT_IP, "front_laser")
    rear = sick_node("rear_scanner", REAR_IP, "rear_laser")

    merger = ComposableNodeContainer(
        name="laser_merger_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="dual_laser_merger",
                plugin="merger_node::MergerNode",
                name="dual_laser_merger",
                parameters=[{
                    "laser_1_topic": "/front_scanner/scan",
                    "laser_2_topic": "/rear_scanner/scan",
                    "merged_scan_topic": "/scan_merged",
                    "target_frame": TARGET_FRAME,
                    # TF를 정확히 줬으면 오프셋은 0으로 두기 (미세 보정용)
                    "laser_1_x_offset": 0.0,
                    "laser_1_y_offset": 0.0,
                    "laser_1_yaw_offset": 0.0,
                    "laser_2_x_offset": 0.0,
                    "laser_2_y_offset": 0.0,
                    "laser_2_yaw_offset": 0.0,
                    "tolerance": 0.1,
                    "queue_size": 5,
                    "angle_increment": 0.0024,   # nanoScan3 각분해능 근처
                    "scan_time": 0.04,           # 25Hz
                    "range_min": 0.05,
                    "range_max": 40.0,
                    "min_height": -1.0,
                    "max_height": 1.0,
                    "angle_min": -3.141592654,
                    "angle_max": 3.141592654,
                    "inf_epsilon": 1.0,
                    "use_inf": True,
                    "allowed_radius": 0.0,       # 로봇 몸체 반경 필터. 필요하면 키우기
                    "enable_shadow_filter": True,
                    "enable_average_filter": True,
                }],
            )
        ],
        output="screen",
    )

    return LaunchDescription([
        static_tf("front_laser", FRONT_POSE),
        static_tf("rear_laser", REAR_POSE),
        front,
        rear,
        merger,
    ])
