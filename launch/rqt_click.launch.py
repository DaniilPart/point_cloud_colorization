import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    package_share = get_package_share_directory('pointcloud_colorizer')
    perspective_file = os.path.join(package_share, 'rqt', 'Click.perspective')

    rqt = ExecuteProcess(
        cmd=['rqt', '--perspective-file', perspective_file],
        output='screen',
    )

    return LaunchDescription([rqt])
