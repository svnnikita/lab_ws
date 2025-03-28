/* Подключаем заголовочный файл, который относится к клиентской библиотеке
 * rclcpp.hpp. Она обеспечивает функционал ROS2 */
#include <rclcpp/rclcpp.hpp>

// Подключаем библиотеку строковых сообщений
//#include <std_msgs/msg/string.hpp>

// Подключаем библиотеку для отображения сообщений типа Int32
#include <std_msgs/msg/int32.hpp>

// Указываем пространство имен для указания единиц измерения времени
using namespace std::chrono_literals;

// ======================================================================
// Подписчик будет мониторить данные из топика /msg_tr, данные в которые
// отправляет узел публикатора lab1_basic_publisher_1
// ======================================================================
class TestBasicSubscriber : public rclcpp::Node
{
public:
    TestBasicSubscriber():
        Node("test_basic_subscriber_node") // название узла

    {
        subscription = this->create_subscription<std_msgs::msg::Int32>
                       ("/msg_tr", 10,
                        std::bind(&TestBasicSubscriber::topic_callback,
                                  this, std::placeholders::_1));
    }
private:
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription;
    void topic_callback(const std_msgs::msg::Int32 &msg) {
        // Выводим в консоль то, что опубликовали (для проверки)
        RCLCPP_INFO_STREAM(this->get_logger(), "Received: " << msg.data);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);                       // подключаемся к системе, передавая
        // аргументы командной строки

    rclcpp::spin(std::make_shared<TestBasicSubscriber>());     // запуск на исполнение

    rclcpp::shutdown();                             // завершаем работу узла
    return 0;
}
