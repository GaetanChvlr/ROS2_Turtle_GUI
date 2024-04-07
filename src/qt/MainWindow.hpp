#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QMainWindow>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <QTimer>
#include <thread>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

enum Mode {INACTIF, ACTIF, CIBLE};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    void publishMessage();
    ~MainWindow();

private slots:
    void onStartBtnClick();
    void onStopBtnClick();
    void onGoBtnClick();
    void onSpeedValueChange(float val);
    void poseCallback(const turtlesim::msg::Pose::SharedPtr msg);

private:
    Ui::MainWindow *ui;
    rclcpp::Node::SharedPtr ros_node;
    std::thread ros_thread;
    rclcpp::executors::SingleThreadedExecutor executor;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber;
    turtlesim::msg::Pose current_pose;
    QTimer *publish_timer;
    bool running;
    Mode current_mode;
};

#endif // MAINWINDOW_HPP
