// Подключаем заголовочный файл, который относится к клиентской библиотеке
// rclcpp.hpp. Она обеспечивает функционал ROS2
#include <rclcpp/rclcpp.hpp>

// Подключаем библиотеку строковых сообщений
//#include <std_msgs/msg/string.hpp>

// Подключаем библиотеку для отображения сообщений типа Int32
#include <std_msgs/msg/int32.hpp>

// Указываем пространство имен для указания единиц измерения времени
using namespace std::chrono_literals;

// Создаем класс узла.
// Публикатор публикует сообщение типа Int32 в топик /msg_tr
class TestBasicPublisher : public rclcpp::Node
{
public:
    TestBasicPublisher():
        Node("lab1_basic_publisher_1_node"), // название узла
        counter(0)
    {
        // Публикатор публикует сообщение типа Int32 в топик /msg_tr
        publisher = this->create_publisher<std_msgs::msg::Int32>("/msg_tr", 10);
        timer = this->create_wall_timer(1s, std::bind(&TestBasicPublisher::timer_callback, this));
    }
private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher;
    unsigned int counter;

    void timer_callback() {
        std_msgs::msg::Int32 message;   // создаем объект, который
                                        // отвечает за сообщения

        message.data = counter;         // создаем строку

        publisher->publish(message);                            // публикуем сообщение

        // Выводим в консоль то, что опубликовали (для проверки)
        RCLCPP_INFO_STREAM(this->get_logger(), "Published: " << message.data);
        counter++;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);                       // подключаемся к системе, передавая
                                                    // аргументы командной строки

    rclcpp::spin(std::make_shared<TestBasicPublisher>());     // запуск на исполнение

    rclcpp::shutdown();                             // завершаем работу узла
    return 0;
}
