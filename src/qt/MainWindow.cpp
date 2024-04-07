#include "MainWindow.hpp"
#include "ui_MainWindow.h"
#include <iomanip>
#include <sstream>

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

    // Liaison scrollBar - fonction
    connect(ui->speedScrollBar, &QScrollBar::valueChanged, this, &MainWindow::onSpeedValueChange);

    // Position cible
    ui->targetX->setRange(0,11);
    ui->targetY->setRange(0,11);

    // Etat initial vitesse
    ui->speedScrollBar->setValue(50);

    // Etat initial labels
    ui->label_run->hide();
    ui->label_stop->show();

}

MainWindow::~MainWindow() {
    delete ui;
}

// Lecture pose courante
void MainWindow::poseCallback(const turtlesim::msg::Pose::SharedPtr msg) {
    current_pose = *msg;
    float theta_to_deg = current_pose.theta * 180.0 / 3.14159;

    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2);
    stream << "X: " << current_pose.x;
    stream << " ,Y: " << current_pose.y;
    stream << " ," << u8"\u03B8" << ": " << theta_to_deg;

    ui->label_pos->setText(QString::fromStdString(stream.str()));
}

// Publication de message avec timer actif
void MainWindow::publishMessage() {
    auto message = geometry_msgs::msg::Twist();
    float currentSpeed = ui->speedScrollBar->value() / 25.0;

    switch(current_mode) {
        case ACTIF: {        // CERCLE
            message.linear.x = 1.0 * currentSpeed;  
            message.angular.z = 1.0 * currentSpeed; 

            ui->label_run->show();
            ui->label_stop->hide();

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

            ui->label_run->show();
            ui->label_stop->hide();

            if (std::abs(ang_err) > 0.2) {
                message.angular.z = k_ang * ang_err * currentSpeed; 
            } 
            else if (d > 0.1) {
                message.linear.x = k_lin * d * currentSpeed;
            }
            else {
                current_mode = INACTIF;
                publish_timer->stop(); 

                ui->label_run->hide();
                ui->label_stop->show();
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
    publish_timer->start(100);
}

// Arrêt
void MainWindow::onStopBtnClick() {
    current_mode = INACTIF;
    publish_timer->stop();

    ui->label_run->hide();
    ui->label_stop->show();

    auto stop_message = geometry_msgs::msg::Twist();
    stop_message.linear.x = 0.0;
    stop_message.angular.z = 0.0;
    velocity_publisher->publish(stop_message);
}

// Deplacement vers position cible
void MainWindow::onGoBtnClick(){
    current_mode = CIBLE;
    publish_timer->start(100);
}

// Modification de la vitesse
void MainWindow::onSpeedValueChange(float val){
    float currentSpeed = val / 50.0;

    std::ostringstream stream;
    stream << "x " << currentSpeed;
    ui->label_speed->setText(QString::fromStdString(stream.str()));
}

