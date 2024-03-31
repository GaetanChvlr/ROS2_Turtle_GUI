#include "MainWindow.hpp"
#include "ui_MainWindow.h"

MainWindow::MainWindow(QWidget *parent) 
    : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);

    // Node ROS2 pour publier sur cmd_vel
    ros_node = rclcpp::Node::make_shared("qt_ros_node");
    velocity_publisher = ros_node->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

    // Timer
    publish_timer = new QTimer(this);
    connect(publish_timer, &QTimer::timeout, this, &MainWindow::publishMessage);

    // Etat de la tortue
    running = false;

    // Liaison boutons - fonctions
    connect(ui->startButton, &QPushButton::clicked, this, &MainWindow::onStartBtnClick);
    connect(ui->stopButton, &QPushButton::clicked, this, &MainWindow::onStopBtnClick);
}

MainWindow::~MainWindow() {
    delete ui;
}

// Publication de message avec timer actif
void MainWindow::publishMessage() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 1.0;  
    message.angular.z = 1.0; 
    velocity_publisher->publish(message);
}

// Demarrage
void MainWindow::onStartBtnClick() {
    running = true;
    publish_timer->start(100);
}

// Arrêt
void MainWindow::onStopBtnClick() {
    running = false;
    publish_timer->stop();

    auto stop_message = geometry_msgs::msg::Twist();
    stop_message.linear.x = 0.0;
    stop_message.angular.z = 0.0;
    velocity_publisher->publish(stop_message);
}


