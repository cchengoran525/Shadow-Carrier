"""offline_x3pro_launch.py —— 用 X3Pro 定制配置跑 slam_toolbox 离线（sync）节点

与官方 offline_launch.py 的差别只有一处：配置指向本目录的 mapper_params_x3pro.yaml
（官方 launch 硬编码 mapper_params_offline.yaml 且不收参数）。
用法（配合 bag 回放）:
  ros2 launch offline_x3pro_launch.py &
  ros2 bag play <bag> --clock
"""
import os

from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    here = os.path.dirname(os.path.abspath(__file__))
    node = LifecycleNode(
        parameters=[os.path.join(here, 'mapper_params_x3pro.yaml'),
                    {'use_sim_time': True}],
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace='',
    )
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
    )
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            start_state='configuring',
            goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(node),
                transition_id=Transition.TRANSITION_ACTIVATE,
            ))],
        ),
    )
    ld = LaunchDescription()
    ld.add_action(node)
    ld.add_action(configure_event)
    ld.add_action(activate_event)
    return ld
