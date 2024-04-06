//#include <rclcpp/rclcpp.h>
#include <QApplication>
#include "qt/MainWindow.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc,argv);        // ROS
    QApplication app(argc, argv);   // QT

    // Turtlesim node en arriere plan
    system("ros2 run turtlesim turtlesim_node &");
    
    MainWindow mainWin;
    mainWin.show();
    int res = app.exec();

    system("killall -q turtlesim_node");
    rclcpp::shutdown();
    return res;
}
