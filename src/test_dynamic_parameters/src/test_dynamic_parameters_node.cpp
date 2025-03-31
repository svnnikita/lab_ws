#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

// Создаем класс узла:
class TestDynamicParameters : public rclcpp::Node
{
public:
    TestDynamicParameters():
        Node("test_dynamic_parameters_node") // название узла.
    {
        // Объявляем параметры - указываем имя и значение по умолчанию:
        this->declare_parameter("string_parameter", "test_string");
        this->declare_parameter("number_parameter", 15.5);

        callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&TestDynamicParameters::parametersCallback,
                      this,
                      std::placeholders::_1));
    }
private:
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    // Если параметр изменен, будет вызываться функция parametersCallback,
    // которая обрабатывает эти изменения:
    rcl_interfaces::msg::SetParametersResult parametersCallback(

        // В качестве аргументов функции передается вектор из параметров
        const std::vector<rclcpp::Parameter> &parameters) {

        // Результат обработки параметров
        rcl_interfaces::msg::SetParametersResult result;

        result.successful = true;
        result.reason = "success";

        // Обработка параметров. В данном узле 2 параметра:
        for (auto &param : parameters) {

            // Если параметр -- это number_parameter, то его текущее значение
            // выводится в командную строку:
            if (param.get_name() == "number_parameter") {
                RCLCPP_INFO_STREAM(this->get_logger(),
                                   "New value for number: " << param.as_double());
            }

            // Аналогично
            if (param.get_name() == "string_parameter") {
                RCLCPP_INFO_STREAM(this->get_logger(),
                                   "New value for string: " << param.as_string());

            }
        }
        return result;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);                       // подключаемся к системе, передавая
        // аргументы командной строки

    rclcpp::spin(std::make_shared<TestDynamicParameters>());     // запуск на исполнение

    rclcpp::shutdown();                             // завершаем работу узла
    return 0;
}
