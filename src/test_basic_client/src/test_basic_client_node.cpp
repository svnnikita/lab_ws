// Подключаем заголовочный файл, который относится к клиентской библиотеке
// rclcpp.hpp. Она обеспечивает функционал ROS2
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

using namespace std::chrono_literals;

class TestBasicClient : public rclcpp::Node
{
public:
    TestBasicClient():
        Node("test_basic_client_node"),

        // Начальное значение flag_to_set устанавливается true.
        // Оно будет меняться по мере работы узла
        flag_to_set(true)
    {
        client =
            this->create_client<std_srvs::srv::SetBool>("bool_service");
        timer =
            this->create_wall_timer(2s, std::bind(&TestBasicClient::timer_callback, this));
    }

private:
    // Указатели на объекты timer (нужен для периодического вызова запросов)
    // и client. Сервис bool_service, который использует объект client, осуществляет
    // смену флага
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client;

    // Конкретное значение флага записывается в flag_to_set
    bool flag_to_set;

    // Функция timer_callback() будет вызываться через определенный интервал
    // времени для обращения к сервису.
    // Необходимости в периодичности (в использовании таймера) здесь нет,
    // это сделано для демонстрации. В реальной ситуации запрос отправляется
    // по мере надобности.
    void timer_callback() {

        // Проверяем доступность сервиса и ожидаем его в случае, если
        // сервис недоступен.
        while (!client->wait_for_service(100ms)) {

            // Если в процессе работы ожидающего механизма произойдут
            // какие-то ошибки то будет выведено следующее сообщение:
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted");
            }

            // Клиент будет ожидать сервис, пока тот недоступен, и выводить
            // следующее сообщение:
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
        }

        // Если с системой все в порядке, то формируется запрос
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();

        request->data = flag_to_set;    // в поле data устанавливается значение флага,
                                        // которое мы собираемся отправить.

        flag_to_set = !flag_to_set;     // для того, чтобы флаг каждый раз менял свое
                                        // значение, это значение меняется на противоположное

        // Осуществляем вызов запроса в асинхронном режиме, при этом не ожидаем от него ответа.
        // Ответ будет поступать отдельно. При поступлении ответа будет вызываться
        // функция response_callback.
        client->async_send_request(request,
                                   std::bind(&TestBasicClient::response_callback,
                                             this,
                                             std::placeholders::_1));
    }

    // Функция response_callback вызывается при получении ответа от сервиса
    void response_callback(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
        auto status = future.wait_for(100ms);
        if (status == std::future_status::ready) {
            RCLCPP_INFO_STREAM(this->get_logger(),
                               "Result: " << future.get()->success
                                          << " Msg: " << future.get()->message);
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);                       // подключаемся к системе, передавая
                                                    // аргументы командной строки

    rclcpp::spin(std::make_shared<TestBasicClient>());     // запуск на исполнение

    rclcpp::shutdown();                             // завершаем работу узла
    return 0;
}
