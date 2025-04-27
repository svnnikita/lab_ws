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
        Node("lab4_tf_listener_node"),

        // Описание объектов указано в privat-части:
        target_frame_front("front_rangefinder_link"),
        target_frame_rear("rear_rangefinder_link"),
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

    // Строки, которые будут хранить исходные фреймы и целевой фрейм, между
    // которыми будет определяться трансформация:
    std::string target_frame_front, target_frame_rear, source_frame;

    // Принимаем даные о трансформации:
    void timer_callback() {

        // В данный объект загружаем необходимую трансформацию
        geometry_msgs::msg::TransformStamped transform_front;
        geometry_msgs::msg::TransformStamped transform_rear;

        try {
            // Указываем, между какими фреймами происходит трансформация:
            transform_front = tf_buffer->lookupTransform(source_frame,
                                                         target_frame_front,
                                                         tf2::TimePointZero);

            transform_rear = tf_buffer->lookupTransform(source_frame,
                                                         target_frame_rear,
                                                         tf2::TimePointZero);

            // Процедура переноса кватерниона в объект для последующего преобразования через
            // матрицу поворота в углы roll, pitch, yaw:
            tf2::Quaternion quaternion_front(
                transform_front.transform.rotation.x,
                transform_front.transform.rotation.y,
                transform_front.transform.rotation.z,
                transform_front.transform.rotation.w);

            tf2::Quaternion quaternion_rear(
                transform_rear.transform.rotation.x,
                transform_rear.transform.rotation.y,
                transform_rear.transform.rotation.z,
                transform_rear.transform.rotation.w);

            tf2::Matrix3x3 matrix_front(quaternion_front);
            tf2::Matrix3x3 matrix_rear(quaternion_rear);


            double roll_front, pitch_front, yaw_front;
            double roll_rear, pitch_rear, yaw_rear;

            matrix_front.getRPY(roll_front, pitch_front, yaw_front);
            matrix_rear.getRPY(roll_rear, pitch_rear, yaw_rear);

            RCLCPP_INFO_STREAM(this->get_logger(),  "\n\n" <<
                   "TRANSFORMATION\n"<< "FRONT_RANGEFINDER: " <<
                               "x: " << transform_front.transform.translation.x <<
                              " y: " << transform_front.transform.translation.y <<
                              " z: " << transform_front.transform.translation.z << "\n" <<
                           " roll: " << roll_front  <<
                          " pitch: " << pitch_front <<
                            " yaw: " << yaw_front   << "\n" <<
                "REAR_RANGEFINDER: " <<
                               "x: " << transform_rear.transform.translation.x <<
                              " y: " << transform_rear.transform.translation.y <<
                              " z: " << transform_rear.transform.translation.z <<  "\n" <<
                           " roll: " << roll_rear  <<
                          " pitch: " << pitch_rear <<
                            " yaw: " << yaw_rear);
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
