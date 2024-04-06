#include "MainWindow.hpp"
#include "ui_MainWindow.h"

MainWindow::MainWindow(QWidget *parent) 
    : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);

    // Node ROS2 pour publier sur cmd_vel et lire la pose
    ros_node = rclcpp::Node::make_shared("qt_ros_node");
    velocity_publisher = ros_node->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
    pose_subscriber = ros_node->create_subscription<turtlesim::msg::Pose>(
        "turtle1/pose", 10, std::bind(&MainWindow::poseCallback, this, std::placeholders::_1)
    );

    executor.add_node(ros_node);
    ros_thread = std::thread([this] {executor.spin();});

    // Timer
    publish_timer = new QTimer(this);
    connect(publish_timer, &QTimer::timeout, this, &MainWindow::publishMessage);

    // Etat de la tortue
    running = false;

    // Liaison boutons - fonctions
    connect(ui->startButton, &QPushButton::clicked, this, &MainWindow::onStartBtnClick);
    connect(ui->stopButton, &QPushButton::clicked, this, &MainWindow::onStopBtnClick);
    connect(ui->goButton, &QPushButton::clicked, this, &MainWindow::onGoBtnClick);

    // Position cible
    ui->targetX->setRange(0,11);
    ui->targetY->setRange(0,11);

}

MainWindow::~MainWindow() {
    delete ui;
}

// Lecture pose courante
void MainWindow::poseCallback(const turtlesim::msg::Pose::SharedPtr msg) {
    current_pose = *msg;
}

// Publication de message avec timer actif
void MainWindow::publishMessage() {
    auto message = geometry_msgs::msg::Twist();

    switch(current_mode) {
        case ACTIF: {        // CERCLE
            message.linear.x = 1.0;  
            message.angular.z = 1.0; 
            break;
        }
        case CIBLE: {         // ATTEINTE POSITION CIBLE

            // Gains
            float k_lin = 0.5;
            float k_ang = 1.5;
            float x_err = ui->targetX->value() - current_pose.x;
            float y_err = ui->targetY->value() - current_pose.y;
            float d = sqrt(x_err*x_err + y_err*y_err);

            // Angle d'orientation
            float ang_cible = atan2(y_err,x_err);
            float ang_err = ang_cible - current_pose.theta;

            if (std::abs(ang_err) > 0.2) {
                message.angular.z = k_ang * ang_err; 
            } 
            else if (d > 0.2) {
                message.linear.x = k_lin * d;
            }
            else {
                current_mode = INACTIF;
                publish_timer->stop(); 
            }
            break;
        }
        default:
            return;

    }
    velocity_publisher->publish(message);
}

// Demarrage
void MainWindow::onStartBtnClick() {
    current_mode = ACTIF;
    running = true;
    publish_timer->start(100);
}

// Arrêt
void MainWindow::onStopBtnClick() {
    current_mode = INACTIF;
    running = false;
    publish_timer->stop();

    auto stop_message = geometry_msgs::msg::Twist();
    stop_message.linear.x = 0.0;
    stop_message.angular.z = 0.0;
    velocity_publisher->publish(stop_message);
}

// Deplacement vers position cible
void MainWindow::onGoBtnClick(){
    current_mode = CIBLE;
    running = true;
    publish_timer->start(100);
}

