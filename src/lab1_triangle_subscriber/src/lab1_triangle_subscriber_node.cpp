/* Подключаем заголовочный файл, который относится к клиентской библиотеке
 * rclcpp.hpp. Она обеспечивает функционал ROS2 */
#include <rclcpp/rclcpp.hpp>

// Подключаем файл с сообщениями
#include <lab1_triangle_msg/msg/triangle.hpp>

#include <math.h>

// Указываем пространство имен для указания единиц измерения времени
using namespace std::chrono_literals;


// Создаем класс.
// Подписчик мониторит данные из топика /triangle, данные в которые
// отправляет узел публикатора lab1_triangle_publisher
class TriangleSubscriber : public rclcpp::Node
{
public:
    TriangleSubscriber():
        Node("lab1_triangle_subscriber_node") // название узла

    {
        subscription = this->create_subscription<lab1_triangle_msg::msg::Triangle>
                       ("/triangle", 10,
                        std::bind(&TriangleSubscriber::triangle_callback,
                                  this, std::placeholders::_1));
    }
private:
    rclcpp::Subscription<lab1_triangle_msg::msg::Triangle>::SharedPtr subscription;

    // Создаем переменные, которые хранят значения длин векторов
    double vector_AB;
    double vector_BC;
    double vector_CA;

    // Полупериметр и площадь треугольника
    double semiperimeter;
    double square;

    // Шаблон функции, возвращающей длину вектора
    template<typename coordinate>
    double vector_length(coordinate vertex1_X, coordinate vertex1_Y,
                         coordinate vertex2_X, coordinate vertex2_Y) {
        return sqrt((vertex2_X - vertex1_X) * (vertex2_X - vertex1_X)
                    + (vertex2_Y - vertex1_Y) * (vertex2_Y - vertex1_Y));
    }

    void triangle_callback(const lab1_triangle_msg::msg::Triangle &msg_tr) {

        vector_AB = vector_length(msg_tr.vertexa.x, msg_tr.vertexa.y,
                                 msg_tr.vertexb.x, msg_tr.vertexb.y);

        vector_BC = vector_length(msg_tr.vertexb.x, msg_tr.vertexb.y,
                                  msg_tr.vertexc.x, msg_tr.vertexc.y);

        vector_CA = vector_length(msg_tr.vertexc.x, msg_tr.vertexc.y,
                                  msg_tr.vertexa.x, msg_tr.vertexa.y);

        semiperimeter = (vector_AB + vector_BC + vector_CA)/2;
        square = sqrt(semiperimeter * (semiperimeter - vector_AB) *
                      (semiperimeter - vector_BC) * (semiperimeter - vector_CA));

        // Выводим в консоль то, что опубликовали (для проверки)
        RCLCPP_INFO_STREAM(this->get_logger(), "Square is: " << square);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);                       // подключаемся к системе, передавая
        // аргументы командной строки

    rclcpp::spin(std::make_shared<TriangleSubscriber>());     // запуск на исполнение

    rclcpp::shutdown();                             // завершаем работу узла
    return 0;
}
