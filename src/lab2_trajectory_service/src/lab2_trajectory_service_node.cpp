#include <rclcpp/rclcpp.hpp>
#include <lab2_trajectory_srv/srv/trajectory.hpp>
#include <math.h>

// Указываем пространство имен для указания единиц измерения времени.
using namespace std::chrono_literals;

class TestTrajectoryService : public rclcpp::Node
{
public:
    TestTrajectoryService():
        Node("lab2_trajectory_service_node") // название узла,
    {
        service =
            this->create_service<lab2_trajectory_srv::srv::Trajectory>("trajectory",
                                                                       std::bind(&TestTrajectoryService::calculateLength,
                                                                                 this,
                                                                                 std::placeholders::_1,
                                                                                 std::placeholders::_2));
    }
private:
    rclcpp::Service<lab2_trajectory_srv::srv::Trajectory>::SharedPtr service;

    // Шаблон функции, возвращающей длину вектора (расстояние
    // между точками) по трем координатам (в пространстве):
    template<typename coordinate>
    double euclideanDistance(coordinate vertex1_X, coordinate vertex1_Y, coordinate vertex1_Z,
                             coordinate vertex2_X, coordinate vertex2_Y, coordinate vertex2_Z) {
        return sqrt((vertex2_X - vertex1_X) * (vertex2_X - vertex1_X)
                    + (vertex2_Y - vertex1_Y) * (vertex2_Y - vertex1_Y)
                    + (vertex2_Z - vertex1_Z) * (vertex2_Z - vertex1_Z));
    }

    // Функция, вычисляющая длину траектории по четырем точкам:
    void calculateLength(const std::shared_ptr<lab2_trajectory_srv::srv::Trajectory::Request> request,
                         std::shared_ptr<lab2_trajectory_srv::srv::Trajectory::Response> response) {

        // Расстояние между 1-й и 2-й точкой:
        double length1 = euclideanDistance(request->pt1.x, request->pt1.y, request->pt1.z,
                                           request->pt2.x, request->pt2.y, request->pt2.z);

        // Расстояние между 2-й и 3-й точкой:
        double length2 = euclideanDistance(request->pt2.x, request->pt2.y, request->pt2.z,
                                           request->pt3.x, request->pt3.y, request->pt3.z);

        // Расстояние между 3-й и 4-й точкой:
        double length3 = euclideanDistance(request->pt3.x, request->pt3.y, request->pt3.z,
                                           request->pt4.x, request->pt4.y, request->pt4.z);

        // Ответ - длина траектории:
        response->length = length1 + length2 + length3;

        // Выводим в консоль то, что опубликовали (для проверки).
        RCLCPP_INFO_STREAM(this->get_logger(), "Length of trajectory: " << response->length);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);                       // подключаемся к системе, передавая
        // аргументы командной строки.

    rclcpp::spin(std::make_shared<TestTrajectoryService>());     // запуск на исполнение.
    rclcpp::shutdown();
    return 0;
}

