from launch import LaunchDescription
from launch_ros.actions import Node

# Функция возвращает конфигурацию запуска, в которую входит
# описание узлов:
def generate_launch_description():
    return LaunchDescription([
        Node(

            # Пакет, к которому относится узел:
            package = "test_parameters",

            # Исполняемый файл:
            executable = "test_parameters_node",

            # Имя для узла. Имена могут быть различными, поскольку можно
            # запустить несколько одинаковых узлов:
            name = "test_param_node_1",

            # Выводим всю консольную информацию на экран:
            output = "screen",

            # Указываем значения параметров:
            parameters = [
                {"string_parameter": "launch_test_1",
                "number_parameter": 439.34}
            ]
        ),

        Node(
            package = "test_parameters",
            executable = "test_parameters_node",
            name = "test_param_node_2",
            output = "screen",
            parameters = [
                {"string_parameter": "launch_test_2",
                "number_parameter": 73.43}
            ]
        )
    ])
