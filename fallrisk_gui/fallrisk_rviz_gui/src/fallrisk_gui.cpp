#include "fallrisk_gui.h"
#include "ui_fallrisk_gui.h"

#include <iostream>
#include <sensor_msgs/Image.h>

FallRiskGUI::FallRiskGUI(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::FallRiskGUI),it_(nh_)
{
    ui->setupUi(this);
    ui->sliderLinearVel->setValue(75);
    ui->sliderAngularVel->setValue(75);

    initVariables();
    initDisplayWidgets();
    initActionsConnections();

    //just for testing, needs to be commented out
    cv::namedWindow("Image window");
}

FallRiskGUI::~FallRiskGUI()
{
    delete ui;
    cv::destroyWindow("Image window");
}

void FallRiskGUI::initVariables()
{
    moveBaseCmdPub = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
    centerDistSub = nh_.subscribe("/distance/image_center_dist",1,&FallRiskGUI::distanceSubCallback,this);
    baseSensorStatus = nh_.subscribe("/mobile_base/sensors/core",1,&FallRiskGUI::baseStatusCheck,this);

    liveVideoSub = it_.subscribe("/camera/rgb/image_raw",1,&FallRiskGUI::liveVideoCallback,this);

    setRobotVelocity();
}

void FallRiskGUI::initActionsConnections()
{
    connect(ui->btnUp, SIGNAL(clicked()), this, SLOT(moveBaseForward()));
    connect(ui->btnDown, SIGNAL(clicked()), this, SLOT(moveBaseBackward()));
    connect(ui->btnLeft, SIGNAL(clicked()), this, SLOT(moveBaseLeft()));
    connect(ui->btnRight, SIGNAL(clicked()), this, SLOT(moveBaseRight()));

    connect(ui->sliderLinearVel, SIGNAL(valueChanged(int)),this,SLOT(setRobotVelocity()));
    connect(ui->sliderAngularVel, SIGNAL(valueChanged(int)),this,SLOT(setRobotVelocity()));
}

void FallRiskGUI::initDisplayWidgets()
{
    // Initialize GUI elements
    render_panel_ = new rviz::RenderPanel();
    ui->display3d_layout->addWidget(render_panel_);

    manager_ = new rviz::VisualizationManager( render_panel_ );
    render_panel_->initialize( manager_->getSceneManager(), manager_ );

    //set the fixed frame before initializing Visualization Manager. pointcloud2 will not work with this
    manager_->setFixedFrame("/base_link");
    manager_->initialize();
    manager_->startUpdate();

    // Create a main display.
    mainDisplay_ = manager_->createDisplay( "rviz/PointCloud2", "Image View", true );
    ROS_ASSERT( mainDisplay_ != NULL );

    mainDisplay_->subProp( "Topic" )->setValue( "/camera/depth/points" );
    mainDisplay_->subProp( "Selectable" )->setValue( "true" );
    mainDisplay_->subProp( "Style" )->setValue( "Boxes" );
//    mainDisplay_->subProp( "Size" )->setValue( 0.01 );
    mainDisplay_->subProp("Alpha")->setValue(1);

//    imagePanel_=new rviz::Panel();
//    imagePanel_->initialize(manager_);
//    imageDisplay_ = manager_->createDisplay( "rviz/Image", "Image View", true );
//    imageDisplay_->subProp("Topic")->setValue("/camera/rgb/image_raw");
//    ui->livevideo_layout->addWidget(imagePanel_);

    /*
    //Image :
        grid_ = manager_->createDisplay( "rviz/Image", "Image View", true );
        ROS_ASSERT( grid_ != NULL );
        grid_->subProp( "Image Topic" )->setValue( "/camera/rgb/image_raw" );
        grid_->subProp( "Transport Hint" )->setValue( "theora" );


    //Depth Cloud :
        grid_ = manager_->createDisplay( "rviz/DepthCloud", "Image View", true );
        ROS_ASSERT( grid_ != NULL );

        grid_->subProp( "Depth Map Topic" )->setValue( "/camera/depth/image_raw" );
        grid_->subProp( "Depth Map Transport Hint" )->setValue( "raw" );
        grid_->subProp( "Color Image Topic" )->setValue( "/camera/rgb/image_raw" );
        grid_->subProp( "Color Transport Hint" )->setValue( "raw" );
        grid_->subProp("Queue Size")->setValue(5);
        grid_->subProp("Style")->setValue("Flat Squares");

    //PointCloud2 :
        grid_ = manager_->createDisplay( "rviz/PointCloud2", "Image View", true );
        ROS_ASSERT( grid_ != NULL );

        grid_->subProp( "Topic" )->setValue( "/camera/depth/points" );
        grid_->subProp( "Selectable" )->setValue( "true" );
        grid_->subProp( "Style" )->setValue( "Boxes" );
        grid_->subProp( "Size" )->setValue( 0.01 );
        grid_->subProp("Alpha")->setValue(1);
    */

}

void FallRiskGUI::keyPressEvent(QKeyEvent *event)
{
    switch(event->key())
    {
    case Qt::Key_W:
        moveBaseForward();
        sendMoveBaseCmd();
        ROS_INFO("key W pressed");
        break;
    case Qt::Key_A:
        moveBaseLeft();
        sendMoveBaseCmd();
        ROS_INFO("key A pressed");
        break;
    case Qt::Key_D:
        moveBaseRight();
        sendMoveBaseCmd();
        ROS_INFO("key D pressed");
        break;
    case Qt::Key_S:
        moveBaseBackward();
        sendMoveBaseCmd();
        ROS_INFO("key S pressed");
        break;
//    case Qt::Key_Q:
//        move_in();
//        break;
//    case Qt::Key_E:
//        move_out();
//        break;
    default:
        QWidget::keyPressEvent(event);
        break;
    }
}

void FallRiskGUI::distanceSubCallback(const std_msgs::Float32::ConstPtr& msg)
{
////    ROS_INFO("distance: %f",msg->data);
//    QLocale german(QLocale::German, QLocale::Germany);
//    QString qdist = german.toString(msg->data, 'f', 2);
//    ui->lbDistance->setText(qdist);
}

void FallRiskGUI::baseStatusCheck(const kobuki_msgs::SensorState::ConstPtr& msg)
{
    /*---------- battery of kobuki base -----------*/
//    ROS_INFO("battery: %d",msg->battery);

    int battery_percentage = 0;

    battery_percentage = (msg->battery - BASE_BATTERY_DANGER)*100/(BASE_BATTERY_CAP-BASE_BATTERY_DANGER);
    ui->pbBaseBattery->setValue(battery_percentage);
//    if(msg->battery <= BASE_BATTERY_LOW)
//    QPalette p = ui->pbBaseBattery->palette();
//        p.setColor(QPalette::Highlight, Qt::red);
//        ui->pbBaseBattery->setPalette(p);
//    else
        //ui->pbBaseBattery->setStyleSheet(safe);

    /*-------------- bumper sensors ---------------*/
    if(msg->bumper == msg->BUMPER_LEFT)
    {
        ROS_INFO("BUMPER LEFT");
        ui->btnBumperLeft->setAutoFillBackground(true);
        ui->btnBumperLeft->setStyleSheet(("background-color: rgb(255, 0, 0); color: rgb(255, 255, 255)"));
    }
    else if(msg->bumper == msg->BUMPER_CENTRE)
    {
        ROS_INFO("BUMPER CENTER");
        ui->btnBumperCenter->setAutoFillBackground(true);
        ui->btnBumperCenter->setStyleSheet(("background-color: rgb(255, 0, 0); color: rgb(255, 255, 255)"));

    }
    else if(msg->bumper == msg->BUMPER_RIGHT)
    {
        ROS_INFO("BUMPER RIGHT");
        ui->btnBumperRight->setAutoFillBackground(true);
        ui->btnBumperRight->setStyleSheet(("background-color: rgb(255, 0, 0); color: rgb(255, 255, 255)"));
    }
    else
    {
        ui->btnBumperLeft->setStyleSheet(("background-color: rgb(0, 204, 102); color: rgb(255, 255, 255)"));
        ui->btnBumperCenter->setStyleSheet(("background-color: rgb(0, 204, 102); color: rgb(255, 255, 255)"));
        ui->btnBumperRight->setStyleSheet(("background-color: rgb(0, 204, 102); color: rgb(255, 255, 255)"));
    }
}

void FallRiskGUI::liveVideoCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::imshow("Image window", cv_ptr->image);
    cv::waitKey(3);
}

void FallRiskGUI::setRobotVelocity()
{
    linearVelocity = ui->sliderLinearVel->value()*(LIN_VEL_MAX-LIN_VEL_MIN)/100+LIN_VEL_MIN;
    ROS_INFO("Linear velocity:%f",linearVelocity);
    angularVelocity = ui->sliderAngularVel->value()*(ANG_VEL_MAX-ANG_VEL_MIN)/100+ANG_VEL_MIN;
    ROS_INFO("Angular velocity:%f",angularVelocity);
}

void FallRiskGUI::moveBaseForward()
{
    ROS_INFO("move forward");

    moveBaseCmd.linear.x=linearVelocity;
    moveBaseCmd.linear.y=0;
    moveBaseCmd.linear.z=0;

    moveBaseCmd.angular.x=0;
    moveBaseCmd.angular.y=0;
    moveBaseCmd.angular.z=0;

}

void FallRiskGUI::moveBaseBackward()
{
    ROS_INFO("move backward");

    moveBaseCmd.linear.x=-linearVelocity;
    moveBaseCmd.linear.y=0;
    moveBaseCmd.linear.z=0;

    moveBaseCmd.angular.x=0;
    moveBaseCmd.angular.y=0;
    moveBaseCmd.angular.z=0;
}

void FallRiskGUI::moveBaseLeft()
{
    ROS_INFO("move left");

    moveBaseCmd.linear.x=0;
    moveBaseCmd.linear.y=0;
    moveBaseCmd.linear.z=0;

    moveBaseCmd.angular.x=0;
    moveBaseCmd.angular.y=0;
    moveBaseCmd.angular.z=angularVelocity;
}

void FallRiskGUI::moveBaseRight()
{
    ROS_INFO("move right");

    moveBaseCmd.linear.x=0;
    moveBaseCmd.linear.y=0;
    moveBaseCmd.linear.z=0;

    moveBaseCmd.angular.x=0;
    moveBaseCmd.angular.y=0;
    moveBaseCmd.angular.z=-angularVelocity;
}

void FallRiskGUI::sendMoveBaseCmd()
{
    if(ros::ok() && moveBaseCmdPub)
    {
        moveBaseCmdPub.publish(moveBaseCmd);
        ROS_INFO("move base cmd sent");
    }
}
