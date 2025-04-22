from launch import LaunchDescription
from launch_ros.actions import Node

# Функция возвращает конфигурацию запуска, в которую входит
# описание узлов:
def generate_launch_description():
    return LaunchDescription([
        # Узел для публикации динамической трансформации:
        Node(
            # Название пакета:
            package = "test_tf_broadcaster",

            # Название исполняемого файла:
            executable = "test_tf_broadcaster_node",

            # Имя самого узла:
            name = "test_tf_broadcaster_node",

            # Вывод информации на экран
            output = "screen",
        ),
        Node(
            # НЕ КАСТОМНЫЙ узел из пакета tf2_ros:
            package = "tf2_ros",

            # Название пакета:
            executable = "static_transform_publisher",

            # Передаем аргументы. Аргументы касаются параметров
            # статической трансформации:
            arguments = [

            # x, y, z -- смещение; yaw, pitch, roll -- углы поворота
            # вокруг соответствующих осей:
            '--x', '0.9',
            '--y', '0.1',
            '--z', '0',
            '--yaw', '0.1',
            '--pitch', '0',
            '--roll', '0',

            # Параметры систем координат:
            '--frame-id', 'base_link',          # базовая система координат.
            '--child-frame-id', 'sensor_link'   # система координат датчика.
            ]
        )
    ])
