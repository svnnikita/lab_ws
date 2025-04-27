from launch import LaunchDescription
from launch_ros.actions import Node

# Функция возвращает конфигурацию запуска, в которую входит
# описание узлов:
def generate_launch_description():
    return LaunchDescription([
        # Узел для публикации динамической трансформации:
        Node(
            # Название пакета:
            package = "lab4_tf_broadcaster",

            # Название исполняемого файла:
            executable = "lab4_tf_broadcaster_node",

            # Имя самого узла:
            name = "lab4_tf_broadcaster_node",

            # Вывод информации на экран
            output = "screen",
        ),
        Node(
            # НЕ КАСТОМНЫЙ узел из пакета tf2_ros:
            package = "tf2_ros",

            # Название пакета:
            executable = "static_transform_publisher",

            name = "static_transform_publisher_1",

            # Передаем аргументы. Аргументы касаются параметров
            # статической трансформации:
            arguments = [

            # x, y, z -- смещение; yaw, pitch, roll -- углы поворота
            # вокруг соответствующих осей:
            '--x', '0.5',
            '--y', '0.0',
            '--z', '0.2',

            '--yaw', '0.0',
            '--pitch', '0.0',
            '--roll', '0.0',

            # Параметры систем координат:
            '--frame-id', 'base_link',                      # базовая система координат.
            '--child-frame-id', 'front_rangefinder_link'    # система координат датчика.
            ]
        ),
        Node(
            package = "tf2_ros",
            executable = "static_transform_publisher",
            name = "static_transform_publisher_2",
            arguments = [

            '--x', '-0.4',
            '--y', '0.0',
            '--z', '0.3',

            '--yaw', '3.14',
            '--pitch', '0.0',
            '--roll', '0.0',

            '--frame-id', 'base_link',                   # базовая система координат.
            '--child-frame-id', 'rear_rangefinder_link' # система координат датчика.
            ]
        )
    ])
