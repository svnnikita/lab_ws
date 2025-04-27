#include <rclcpp/rclcpp.hpp>

// Файл описывает структуру сообщений transform_stamped.
// Он публикуется в качестве трансформации:
#include <geometry_msgs/msg/transform_stamped.hpp>

// Файл необходим для публикации трансформаций:
#include <tf2_ros/transform_broadcaster.h>

// Модуль, позволяющий работать с кватернионами как с основным
// способом представления угловых вращений:
#include <tf2/LinearMath/Quaternion.h>

#include <math.h>

using namespace std::chrono_literals;

class TestTfBroadcaster : public rclcpp::Node
{
public:
    TestTfBroadcaster():
        Node("lab4_tf_broadcaster_node"),
        r_orbit(2.0),
        theta_orbit(0.0)
    {
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        timer = this->create_wall_timer(50ms, std::bind(&TestTfBroadcaster::timer_callback, this));
    }
private:
    rclcpp::TimerBase::SharedPtr timer;

    // Указатель на объект tf_broadcaster -- публикатор трансформаций (широковещатель
    // трансформации, который распространяет данные преобразования в пространстве
    // между всеми узлами):
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    double r_orbit, theta_orbit, yaw;

    // Функция вычисления угла рыскания (ориентации мобильной платформы в пространстве):
    double yaw_calc(double amplitude, double theta) {

        double dx = amplitude * cos(theta);
        double dy = 2 * amplitude * cos(2 * theta);

        return atan2(dy, dx);
    }

    // Публикуем трансформацию:
    void timer_callback() {

        geometry_msgs::msg::TransformStamped t;

        // Указываем момент времени получения информации:
        t.header.stamp = this->get_clock()->now();

        // Описываем трансформацию из статического фрэйма "world"
        // в фрэйм "base_link", который будет двигаться вокруг статического фрэйма:
        // Указываем базовый фрэйм:
        t.header.frame_id = "world";    // неподвижный.

        // Указываем дочерний фрейм:
        t.child_frame_id = "base_link";

        /* Задаем параметры трансформации. Первая группа -- ПЕРЕМЕЩЕНИЯ.
         * Перемещение вдоль осей X и Y высчитывается из текущего значения
         * некоторой орбиты r_orbit, по которой система координат будет
         * совершать круговые движения, и текущего углового положения theta_orbit: */
        // t.transform.translation.x = r_orbit * cos(theta_orbit);
        // t.transform.translation.y = r_orbit * sin(theta_orbit);

        // При движении мобильной платформы по восьмерке
        // используются уравнение фигуры Лиссажу при a/b = 1/2:
        t.transform.translation.x = r_orbit * sin(theta_orbit);
        t.transform.translation.y = r_orbit * sin(2 * theta_orbit);

        // Предпологается, что система координат перемещается в горизонтальной
        // плоскости, поэтому перемещение вдоль оси Z равно нулю:
        t.transform.translation.z = 0;

        // Вторая группа -- группа ТРАНСФОРМАЦИЙ, КВАТЕРНИОН, который описывает
        // движение объекта.
        tf2::Quaternion q;

        // При вычислении угла рыскания (yaw) используются производные
        // dx/theta_orbit и dy/theta_orbit. Производные dx/dtheta_orbit
        // и dy/dtheta_orbit — это скорости изменения координат по dx/theta_orbit:
        yaw = yaw_calc(r_orbit, theta_orbit);

        /* Кватернион задается через функцию setRPY(), которая позволяет на основе
         * ориентации углов Тейта-Брайана задать кватернион без вникания
         * в матаппарат данного обеспечения.
         * Т.к. перемещение осуществляется в горизонтальной плоскости, тангаж и крен
         * равны нулю, а рыскание определяется как текущее угловое положение
         * на орбите плюс Пи/2 -- значение поворота на 90 градусов влево: */
        q.setRPY(0, 0, yaw);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        // Отправляем трансформацию в публикатор:
        tf_broadcaster->sendTransform(t);

        theta_orbit += 0.01;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);                       // подключаемся к системе, передавая
        // аргументы командной строки

    rclcpp::spin(std::make_shared<TestTfBroadcaster>());     // запуск на исполнение

    rclcpp::shutdown();                             // завершаем работу узла
    return 0;
}
