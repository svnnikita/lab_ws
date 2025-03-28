#include <rclcpp/rclcpp.hpp>
#include <lab2_trajectory_srv/srv/trajectory.hpp>

using namespace std::chrono_literals;

class TestTrajectoryClient : public rclcpp::Node
{
public:
    TestTrajectoryClient():
        Node("lab2_trajectory_client_node")
    {
        // Создаем объект клиента. Указывается тип сервиса и его название:
        client =
            this->create_client<lab2_trajectory_srv::srv::Trajectory>("trajectory");
        timer =
            this->create_wall_timer(2s, std::bind(&TestTrajectoryClient::timer_callback, this));
    }

private:
    // Указатели на объекты timer (нужен для периодического вызова запросов)
    // и client. Сервис trajectory, который использует объект client, осуществляет
    // вычисление длины траектории
    rclcpp::TimerBase::SharedPtr timer;

    // Создаем указатель на объект клиента. Клиент создается несколькими
    // строками ранее.
    rclcpp::Client<lab2_trajectory_srv::srv::Trajectory>::SharedPtr client;

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
        auto request = std::make_shared<lab2_trajectory_srv::srv::Trajectory::Request>();

        request->pt1.x = 1;	// точка 1
        request->pt1.y = 2;
        request->pt1.z = 3;

        request->pt2.x = 4;	// точка 2
        request->pt2.y = 5;
        request->pt2.z = 6;

        request->pt3.x = 7;	// точка 3
        request->pt3.y = 8;
        request->pt3.z = 9;

        request->pt4.x = 10;	// точка 4
        request->pt4.y = 11;
        request->pt4.z = 12;

        // Осуществляем вызов запроса в асинхронном режиме, при этом не ожидаем от него ответа.
        // Ответ будет поступать ОТДЕЛЬНО. При поступлении ответа будет вызываться
        // функция response_callback.
        client->async_send_request(request,
                                   std::bind(&TestTrajectoryClient::response_callback,
                                             this,
                                             std::placeholders::_1));
    }

    // Функция response_callback вызывается при получении ответа от сервиса.
    // future - указатель, который будет содержать данные ответа от клиента.
    void response_callback(rclcpp::Client<lab2_trajectory_srv::srv::Trajectory>::SharedFuture future) {
        auto status = future.wait_for(100ms);
        if (status == std::future_status::ready) {
            RCLCPP_INFO_STREAM(this->get_logger(),
                               "Result: " << future.get()->length);
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);                       // подключаемся к системе, передавая
        // аргументы командной строки

    rclcpp::spin(std::make_shared<TestTrajectoryClient>());     // запуск на исполнение

    rclcpp::shutdown();                             // завершаем работу узла
    return 0;
}
