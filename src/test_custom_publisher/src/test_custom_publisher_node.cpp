/* Подключаем заголовочный файл, который относится к клиентской библиотеке
 * rclcpp.hpp. Она обеспечивает функционал ROS2 */
#include <rclcpp/rclcpp.hpp>

// Подключаем файл с координатами точек
#include <test_custom_msg/msg/sphere.hpp>

/* Указываем пространство имен для указания единиц измерения времени */
using namespace std::chrono_literals;

// Создаем класс узла
class TestCustomPublisher : public rclcpp::Node
{
public:
    TestCustomPublisher():
        Node("test_custom_publisher_node") // название узла
    {
        publisher = this->create_publisher<test_custom_msg::msg::Sphere>("/test_sphere", 10);
        timer = this->create_wall_timer(500ms, std::bind(&TestCustomPublisher::timer_callback, this));
    }
private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<test_custom_msg::msg::Sphere>::SharedPtr publisher;
    void timer_callback() {
        test_custom_msg::msg::Sphere message;       // создаем объект, который отвечает за сообщения

        message.center.x = 5.0;
        message.center.y = 6.0;
        message.center.z = 7.0;
        message.radius = 75.1;

        publisher->publish(message);                            // публикуем сообщение
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);                       // подключаемся к системе, передавая
                                                    // аргументы командной строки

    rclcpp::spin(std::make_shared<TestCustomPublisher>());     // запуск на исполнение

    rclcpp::shutdown();                             // завершаем работу узла
    return 0;
}
