#include "fallrisk_gui.h"
#include "ui_fallrisk_gui.h"

#include <iostream>
#include <sensor_msgs/Image.h>

FallRiskGUI::FallRiskGUI(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::FallRiskGUI)
{
    ui->setupUi(this);
    ui->sliderLinearVel->setValue(75);
    ui->sliderAngularVel->setValue(75);

    initVariables();
    initDisplayWidgets();
    initActionsConnections();
}

FallRiskGUI::~FallRiskGUI()
{
    delete ui;
}

void FallRiskGUI::initVariables()
{
    moveBaseCmdPub = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
    distSub = nh_.subscribe("/distance/image_center_dist",1,&FallRiskGUI::distanceSubCallback,this);

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
    mainDisplay_->subProp( "Size" )->setValue( 0.01 );
    mainDisplay_->subProp("Alpha")->setValue(1);

    imagePanel_=new rviz::Panel();
    imagePanel_->initialize(manager_);
    imageDisplay_ = manager_->createDisplay( "rviz/Image", "Image View", true );
    imageDisplay_->subProp("Topic")->setValue("/camera/image_raw");
    ui->livevideo_layout->addWidget(imagePanel_);

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
    ROS_INFO("distance: %f",msg->data);
    QLocale german(QLocale::German, QLocale::Germany);
    QString dist = german.toString(msg->data, 'f', 2);
    ui->lbDistance->setText(dist);
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
