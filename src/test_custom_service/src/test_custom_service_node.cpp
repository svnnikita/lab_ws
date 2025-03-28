#include <rclcpp/rclcpp.hpp>
#include <test_custom_srv/srv/triangle_square.hpp>
#include <math.h>

// Указываем пространство имен для указания единиц измерения времени.
using namespace std::chrono_literals;

class TestCustomService : public rclcpp::Node
{
public:
    TestCustomService():
        Node("test_custom_service_node") // название узла,
    {
        service =
            this->create_service<test_custom_srv::srv::TriangleSquare>("triangle_square",
                                                         std::bind(&TestCustomService::calculateSquare,
                                                                   this,
                                                                   std::placeholders::_1,
                                                                   std::placeholders::_2));
    }
private:
    rclcpp::Service<test_custom_srv::srv::TriangleSquare>::SharedPtr service;

    void calculateSquare(const std::shared_ptr<test_custom_srv::srv::TriangleSquare::Request> request,
                 std::shared_ptr<test_custom_srv::srv::TriangleSquare::Response> response) {

        double a = sqrt(pow(request->pt1.x - request->pt2.x, 2) +
                        pow(request->pt1.y - request->pt2.y, 2) +
                        pow(request->pt1.z - request->pt2.z, 2));

        double b = sqrt(pow(request->pt2.x - request->pt3.x, 2) +
                        pow(request->pt2.y - request->pt3.y, 2) +
                        pow(request->pt2.z - request->pt3.z, 2));

        double c = sqrt(pow(request->pt3.x - request->pt1.x, 2) +
                        pow(request->pt3.y - request->pt1.y, 2) +
                        pow(request->pt3.z - request->pt1.z, 2));

        double p = (a + b + c)/2;

        response->square = sqrt(p * (p - a) * (p - b) * (p - c));

        // Выводим в консоль то, что опубликовали (для проверки).
        RCLCPP_INFO_STREAM(this->get_logger(), "Square: " << response->square);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);                       // подключаемся к системе, передавая
                                                    // аргументы командной строки.

    rclcpp::spin(std::make_shared<TestCustomService>());     // запуск на исполнение.
    rclcpp::shutdown();
    return 0;
}

