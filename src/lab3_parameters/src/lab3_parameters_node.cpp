#include <rclcpp/rclcpp.hpp>
#include <math.h>

// Указываем пространство имен для указания единиц измерения времени:
using namespace std::chrono_literals;

// Создаем класс узла:
class TestParameters : public rclcpp::Node
{
public:
    TestParameters():
        Node("lab3_parameters_node") // название узла.
    {
        // Объявляем параметры - указываем имя и значение по умолчанию:
        this->declare_parameter("Vertex1_X", 1.0); // координаты первой вершины треугольника
        this->declare_parameter("Vertex1_Y", 3.0);

        this->declare_parameter("Vertex2_X", 4.0); // координаты второй вершины треугольника
        this->declare_parameter("Vertex2_Y", 6.0);

        this->declare_parameter("Vertex3_X", 7.0); // координаты третьей вершины треугольника
        this->declare_parameter("Vertex3_Y", 8.0);

        // this->declare_parameter("string_parameter", "test_string");
        // this->declare_parameter("number_parameter", 15.5);

        timer = this->create_wall_timer(1s, std::bind(&TestParameters::timer_callback, this));
    }
private:
    rclcpp::TimerBase::SharedPtr timer;

    // Создаем переменные, которые хранят значения длин векторов
    double vector_AB;
    double vector_BC;
    double vector_CA;

    // Полупериметр и площадь треугольника
    double semiperimeter;
    double square;

    // С определенным периодом (1 с) получаем значение параметров, которые задали:
    void timer_callback() {

        double coordA_X = this->get_parameter("Vertex1_X").as_double();
        double coordA_Y = this->get_parameter("Vertex1_Y").as_double();

        double coordB_X = this->get_parameter("Vertex2_X").as_double();
        double coordB_Y = this->get_parameter("Vertex2_Y").as_double();

        double coordC_X = this->get_parameter("Vertex3_X").as_double();
        double coordC_Y = this->get_parameter("Vertex3_Y").as_double();

        // std::string str = this->get_parameter("string_parameter").as_string();  // строковый параметр.
        // double num = this->get_parameter("number_parameter").as_double();   // вещественный параметр.

        vector_AB = vector_length(coordA_X, coordA_Y,
                                  coordB_X, coordB_Y);

        vector_BC = vector_length(coordB_X, coordB_Y,
                                  coordC_X, coordC_Y);

        vector_CA = vector_length(coordC_X, coordC_Y,
                                  coordA_X, coordA_Y);

        semiperimeter = (vector_AB + vector_BC + vector_CA)/2;
        square = sqrt(semiperimeter * (semiperimeter - vector_AB) *
                      (semiperimeter - vector_BC) * (semiperimeter - vector_CA));

        // Выводим в консоль параметры и площадь:
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "\nPARAMETERS:\nA[" << coordA_X << ", "
                                               << coordA_Y << "]; "
                                       << "B[" << coordB_X << ", "
                                               << coordB_Y << "]; "
                                       << "C[" << coordC_X << ", "
                                               << coordC_Y << "];\n"
                                << "SQUARE = " <<  square  << ";\n");
    }

    // Шаблон функции, возвращающей длину вектора
    template<typename coordinate>
    double vector_length(coordinate vertex1_X, coordinate vertex1_Y,
                         coordinate vertex2_X, coordinate vertex2_Y) {
        return sqrt((vertex2_X - vertex1_X) * (vertex2_X - vertex1_X)
                    + (vertex2_Y - vertex1_Y) * (vertex2_Y - vertex1_Y));
    }

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);                       // подключаемся к системе, передавая
                                                    // аргументы командной строки

    rclcpp::spin(std::make_shared<TestParameters>());     // запуск на исполнение

    rclcpp::shutdown();                             // завершаем работу узла
    return 0;
}
