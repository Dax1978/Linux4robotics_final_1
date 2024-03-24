from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Запуск ноды захвата и публикации изображений
        Node(
            package='capture_camera',
            executable='capture_camera',
            name='capture_camera'
        ),
        # Запуск ноды для чтения изображений и вывода их размеров
        Node(
            package='processing_camera',
            executable='processing_camera',
            name='processing_camera'
        ),
        # Запуск ноды для чтения текстовой информации из camera/info
        Node(
            package='processing_info_camera',
            executable='processing_info_camera',
            name='processing_info_camera'
        ),
    ])