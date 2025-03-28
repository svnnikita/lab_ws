// Подключаем заголовочный файл, который относится к клиентской библиотеке
// rclcpp.hpp. Она обеспечивает функционал ROS2
#include <rclcpp/rclcpp.hpp>

// Подключаем библиотеку для отображения сообщений типа Int32
#include <std_msgs/msg/int32.hpp>

// Создаем класс узла.
// Подписчик принимает данные из топика /msg_rcv_1 и отправляет эти
// данные в топик /msg_rcv
class TestReceivingAndSending : public rclcpp::Node
{
public:
    TestReceivingAndSending():
        Node("lab1_rec_sen_node") // название узла
    {
        // Подписчик принимает сообщения типа Int32
        subscription = this->create_subscription<std_msgs::msg::Int32>(
        "/msg_rcv_1", 10, std::bind(&TestReceivingAndSending::rs_callback,
                                  this, std::placeholders::_1));

        // Публиктор отправляет сообщения этого же типа
        publisher = this->create_publisher<std_msgs::msg::Int32>("/msg_rcv", 10);
    }
private:
    // Создаем объекты подписчика и публикатора
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher;

    // Отправляем данные в топик
    void rs_callback(const std_msgs::msg::Int32 &msg) {

        publisher->publish(msg);    // публикуем сообщение

        // Выводим в консоль то, что опубликовали (для проверки)
        RCLCPP_INFO_STREAM(this->get_logger(), "Published and : " << msg.data);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);   // подключаемся к системе, передавая
                                // аргументы командной строки

    rclcpp::spin(std::make_shared<TestReceivingAndSending>());     // запуск на исполнение

    rclcpp::shutdown();         // завершаем работу узла
    return 0;
}
