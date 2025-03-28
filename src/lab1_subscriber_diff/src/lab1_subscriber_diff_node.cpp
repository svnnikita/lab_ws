// Подключаем заголовочный файл, который относится к клиентской библиотеке
// rclcpp.hpp. Она обеспечивает функционал ROS2
#include <rclcpp/rclcpp.hpp>

// Подключаем библиотеку для отображения сообщений типа Int32
#include <std_msgs/msg/int32.hpp>

// Создаем класс узла.
// Подписчик принимает данные из топиков /msg_tr и /msg_rcv и вычисляет разницу.
class TestDifference : public rclcpp::Node
{
public:
    TestDifference():
        Node("test_subscriber_rec_sen_node") // название узла
    {
        // Первый подписчик принимает сообщения из топика /msg_tr
        subscription1 = this->create_subscription<std_msgs::msg::Int32>(
            "/msg_tr", 10, std::bind(&TestDifference::msg_tr_callback,
                      this, std::placeholders::_1));

        // Второй подписчик принимает сообщения из топика /msg_rcv
        subscription2 = this->create_subscription<std_msgs::msg::Int32>(
            "/msg_rcv_1", 10, std::bind(&TestDifference::msg_rcv_callback,
                      this, std::placeholders::_1));

        // Публиктор отправляет сообщения этого же типа
        publisher = this->create_publisher<std_msgs::msg::Int32>("/msg_diff", 10);
    }
private:
    // Создаем объекты подписчиков и публикатора
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription1;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription2;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher;

    // Создаем значения для вычисления разности
    std::optional<int32_t> value1;
    std::optional<int32_t> value2;

    void msg_tr_callback(std_msgs::msg::Int32::SharedPtr msg) {
        value1 = msg->data;
        diff_callback();
    }

    void msg_rcv_callback(std_msgs::msg::Int32::SharedPtr msg) {
        value2 = msg->data;
        diff_callback();
    }

    // Отправляем данные в топик
    void diff_callback() {
        if (value1 && value2) {
            int32_t diff = *value1 - *value2;
            auto diff_msg = std_msgs::msg::Int32();
            diff_msg.data = diff;

            publisher->publish(diff_msg);    // публикуем сообщение

            // Выводим в консоль то, что опубликовали (для проверки)
            RCLCPP_INFO_STREAM(this->get_logger(), "Published: " << diff_msg.data);

            // Сбрасываем значения для следующего расчета
            value1.reset();
            value2.reset();
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);   // подключаемся к системе, передавая
                                // аргументы командной строки

    rclcpp::spin(std::make_shared<TestDifference>());     // запуск на исполнение

    rclcpp::shutdown();         // завершаем работу узла
    return 0;
}
