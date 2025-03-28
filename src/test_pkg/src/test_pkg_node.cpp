
/* Подключаем заголовочный файл, который относится к клиентской библиотеке
 * rclcpp.hpp. Она обеспечивает функционал ROS2 */
#include <rclcpp/rclcpp.hpp>

/* Указываем пространство имен для указания единиц измерения времени */
using namespace std::chrono_literals;

// Создаем класс узла
class TestNode : public rclcpp::Node
{
public:
    TestNode():
        Node("test_node"), // название узла
        counter(0)
    {
        timer = this->create_wall_timer(500ms, std::bind(&TestNode::timer_callback, this));
    }
private:
    rclcpp::TimerBase::SharedPtr timer;
    unsigned int counter;
    void timer_callback() {
        RCLCPP_INFO(this->get_logger(), "Test INFO: %d", counter);
        RCLCPP_INFO_STREAM(this->get_logger(), "Test INFO STREAM: " << counter);
        counter++;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);                       // подключаемся к системе, передавая
                                                    // аргументы командной строки

    rclcpp::spin(std::make_shared<TestNode>());     // запуск на исполнение

    rclcpp::shutdown();                             // завершаем работу узла
    return 0;
}
