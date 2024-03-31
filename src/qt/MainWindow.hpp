#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QMainWindow>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <QTimer>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

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

private:
    Ui::MainWindow *ui;
    rclcpp::Node::SharedPtr ros_node;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher;
    QTimer *publish_timer;
    bool running;
};

#endif // MAINWINDOW_HPP
