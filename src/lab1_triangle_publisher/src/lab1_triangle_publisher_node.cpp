/* Подключаем заголовочный файл, который относится к клиентской библиотеке
 * rclcpp.hpp. Она обеспечивает функционал ROS2 */
#include <rclcpp/rclcpp.hpp>

// Подключаем файл с сообщениями
#include <lab1_triangle_msg/msg/triangle.hpp>

// Указываем пространство имен для указания единиц измерения времени
using namespace std::chrono_literals;

// Создаем класс узла
class TestCustomPublisher1 : public rclcpp::Node
{
public:
    TestCustomPublisher1():
        Node("lab1_triangle_publisher_node") // название узла
    {
        publisher = this->create_publisher<lab1_triangle_msg::msg::Triangle>("/triangle", 10);
        timer = this->create_wall_timer(500ms, std::bind(&TestCustomPublisher1::timer_callback, this));
    }
private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<lab1_triangle_msg::msg::Triangle>::SharedPtr publisher;
    void timer_callback() {
        lab1_triangle_msg::msg::Triangle msg_tr;       // создаем объект, который отвечает за сообщения

        msg_tr.vertexa.x = 2;
        msg_tr.vertexa.y = 3;

        msg_tr.vertexb.x = 7;
        msg_tr.vertexb.y = 4;

        msg_tr.vertexc.x = 5;
        msg_tr.vertexc.y = 7;

        publisher->publish(msg_tr);                            // публикуем сообщение
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);                       // подключаемся к системе, передавая
        // аргументы командной строки

    rclcpp::spin(std::make_shared<TestCustomPublisher1>());     // запуск на исполнение

    rclcpp::shutdown();                             // завершаем работу узла
    return 0;
}
