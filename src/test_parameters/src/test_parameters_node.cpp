#include <rclcpp/rclcpp.hpp>

// Указываем пространство имен для указания единиц измерения времени:
using namespace std::chrono_literals;

// Создаем класс узла:
class TestParameters : public rclcpp::Node
{
public:
    TestParameters():
        Node("test_parameters_node") // название узла.
    {
        // Объявляем параметры - указываем имя и значение по умолчанию:
        this->declare_parameter("string_parameter", "test_string");
        this->declare_parameter("number_parameter", 15.5);

        timer = this->create_wall_timer(1s, std::bind(&TestParameters::timer_callback, this));
    }
private:
    rclcpp::TimerBase::SharedPtr timer;

    // С определенным периодом (1 с) получаем значение параметров, которые задали:
    void timer_callback() {

        // Строковый параметр:
        std::string str = this->get_parameter("string_parameter").as_string();

        // Вещественный параметр:
        double num = this->get_parameter("number_parameter").as_double();

        // Выводим в консоль параметры для проверки:
        RCLCPP_INFO_STREAM(this->get_logger(), "Parameters: str: " << str << " num: " << num);

    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);                       // подключаемся к системе, передавая
                                                    // аргументы командной строки

    rclcpp::spin(std::make_shared<TestParameters>());     // запуск на исполнение

    rclcpp::shutdown();                             // завершаем работу узла
    return 0;
}
