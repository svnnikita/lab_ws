#include <rclcpp/rclcpp.hpp>

// Файл описывает структуру сообщений transform_stamped.
// Он публикуется в качестве трансформации:
#include <geometry_msgs/msg/transform_stamped.hpp>

// Модуль необходим для приема сообщений:
#include <tf2_ros/transform_listener.h>

// Модуль-обработчик исключений, если вдруг трансформация между заданными
// фреймами не будет найдена:
#include <tf2/exceptions.h>

// Модуль позволяет хранить сообщения и играет определенную
// роль в приеме сообщений о трансформациях:
#include <tf2_ros/buffer.h>

// Модуль, позволяющий работать с кватернионами как с основным
// способом представления угловых вращений:
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

class TestTfListener : public rclcpp::Node
{
public:
    TestTfListener():
        Node("test_tf_listener_node"),

        // Описание объектов указано в privat-части:
        target_frame("sensor_link"),
        source_frame("world")
    {
        // Инициализация указателей необходима для того, чтобы: указать tf_listener, в каком буфере
        // хранить трансформации; передавать в буфер механизм реального времени для добавления
        // самой актуальной трансформации:
        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        timer = this->create_wall_timer(500ms, std::bind(&TestTfListener::timer_callback, this));
    }
private:
    rclcpp::TimerBase::SharedPtr timer;

    // Указатель на объект tf_listener позволяет подключаться к модулю публикации и
    // трансформации и регистрировать их в данном узле:
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    // Указатель на объект tf_buffer позволяет накапливать эти трансформации в буфер и
    // в некоторых случаях восстанавливать историю перемещений:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

    // Строки, которые будут хранить исходный фрэйм и целевой фрэйм, между
    // которыми будет определяться трансформация:
    std::string target_frame, source_frame;

    // Принимаем даные о трансформации:
    void timer_callback() {

        // В данный объект загружаем необходимую трансформацию
        geometry_msgs::msg::TransformStamped t;

        try {
            // Указываем, между какими фреймами происходит трансформация:
            t = tf_buffer->lookupTransform(source_frame,
                                           target_frame,
                                           tf2::TimePointZero); // максимально свежая информация.

            // Процедура переноса кватерниона в объект для последующего преобразования через
            // матрицу поворота в углы roll, pitch, yaw:
            tf2::Quaternion q(
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w);

            tf2::Matrix3x3 m(q);

            double roll, pitch, yaw;

            m.getRPY(roll, pitch, yaw);

            RCLCPP_INFO_STREAM(this->get_logger(),
                               "Transformation: " <<
                               "x: " << t.transform.translation.x <<
                              " y: " << t.transform.translation.y <<
                              " z: " << t.transform.translation.z <<
                           " roll: " << roll <<
                          " pitch: " << pitch <<
                            " yaw: " << yaw);
        }

        // Если же в процессе трансформации была допущена ошибка, то в этом
        // случае выводится сообщение с предупреждением:
        catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_STREAM(this->get_logger(),
                               "There are no transform: " << ex.what());
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestTfListener>());
    rclcpp::shutdown();
    return 0;
}
