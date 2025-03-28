#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

// Указываем пространство имен для указания единиц измерения времени.
using namespace std::chrono_literals;

class TestBasicService : public rclcpp::Node
{
public:
    TestBasicService():
        Node("test_basic_service_node"), // название узла,
        flag(false)
    {
        service =
            this->create_service<std_srvs::srv::SetBool>("bool_service",
                                                               std::bind(&TestBasicService::setFlag,
                                                                         this,
                                                                         std::placeholders::_1,
                                                                         std::placeholders::_2));
    }
private:
    // В работе сервиса предпологается, что будет меняться значение флага
    bool flag;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service;

    // request - поле, которое приходит в сервис,
    // response - поле, которое отправляется в качестве ответа.
    void setFlag(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                 std::shared_ptr<std_srvs::srv::SetBool::Response> response) {

        // Устанавливаем flag в значение request.
        flag = request->data;
        response->success = true;
        response->message = "Flag was changed to " + std::string(flag ? "true" : "false");

        // Выводим в консоль то, что опубликовали (для проверки).
        RCLCPP_INFO_STREAM(this->get_logger(), response->message);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);                       // подключаемся к системе, передавая
                                                    // аргументы командной строки.

    rclcpp::spin(std::make_shared<TestBasicService>());     // запуск на исполнение.
    rclcpp::shutdown();
    return 0;
}

