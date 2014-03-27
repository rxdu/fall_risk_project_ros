#include "ui_rviz_gui.h"
#include "rviz_gui.h"
#include <iostream>
#include <sensor_msgs/Image.h>


using namespace std;

RvizGui::RvizGui(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::RvizGui)
{
    ui->setupUi(this);
    ui->sliderLinearVel->setValue(75);
    ui->sliderAngularVel->setValue(75);

    initVariables();
    initDisplayWidgets();
    initActionsConnections();
}

RvizGui::~RvizGui()
{
    delete ui;
}

void RvizGui::initVariables()
{
    moveBaseCmdPub = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);

    setRobotVelocity();
}

void RvizGui::initActionsConnections()
{
    connect(ui->btnUp, SIGNAL(clicked()), this, SLOT(moveBaseForward()));
    connect(ui->btnDown, SIGNAL(clicked()), this, SLOT(moveBaseBackward()));
    connect(ui->btnLeft, SIGNAL(clicked()), this, SLOT(moveBaseLeft()));
    connect(ui->btnRight, SIGNAL(clicked()), this, SLOT(moveBaseRight()));

    connect(ui->sliderLinearVel, SIGNAL(valueChanged(int)),this,SLOT(setRobotVelocity()));
    connect(ui->sliderAngularVel, SIGNAL(valueChanged(int)),this,SLOT(setRobotVelocity()));
}

void RvizGui::initDisplayWidgets()
{
    // Initialize GUI elements
//    render_panel_ = new rviz::RenderPanel();
//    ui->display3d_layout->addWidget(render_panel_);

//    manager_ = new rviz::VisualizationManager( render_panel_ );
//    render_panel_->initialize( manager_->getSceneManager(), manager_ );
//    manager_->initialize();
//    manager_->startUpdate();

//    grid_ = manager_->createDisplay( "rviz/Image", "image display grid", true );
//    ROS_ASSERT( grid_ != NULL );
//    // Create a Grid display.
//    //   grid_ = manager_->createDisplay( "rviz/Grid", "adjustable grid", true );
//    //   ROS_ASSERT( grid_ != NULL );

//    // Configure the GridDisplay the way we like it.
//    // grid_->subProp( "Line Style" )->setValue( "Billboards" );
//    // grid_->subProp( "Color" )->setValue( Qt::yellow );

//    grid_->setTopic("/image_raw","sensor_msgs/Image");

}

void RvizGui::keyPressEvent(QKeyEvent *event)
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

void RvizGui::setRobotVelocity()
{
    linearVelocity = ui->sliderLinearVel->value()*(LIN_VEL_MAX-LIN_VEL_MIN)/100+LIN_VEL_MIN;
    ROS_INFO("Linear velocity:%f",linearVelocity);
    angularVelocity = ui->sliderAngularVel->value()*(ANG_VEL_MAX-ANG_VEL_MIN)/100+ANG_VEL_MIN;
    ROS_INFO("Angular velocity:%f",angularVelocity);
}

void RvizGui::moveBaseForward()
{
    ROS_INFO("move forward");

    moveBaseCmd.linear.x=linearVelocity;
    moveBaseCmd.linear.y=0;
    moveBaseCmd.linear.z=0;

    moveBaseCmd.angular.x=0;
    moveBaseCmd.angular.y=0;
    moveBaseCmd.angular.z=0;

}

void RvizGui::moveBaseBackward()
{
    ROS_INFO("move backward");

    moveBaseCmd.linear.x=-linearVelocity;
    moveBaseCmd.linear.y=0;
    moveBaseCmd.linear.z=0;

    moveBaseCmd.angular.x=0;
    moveBaseCmd.angular.y=0;
    moveBaseCmd.angular.z=0;
}

void RvizGui::moveBaseLeft()
{
    ROS_INFO("move left");

    moveBaseCmd.linear.x=0;
    moveBaseCmd.linear.y=0;
    moveBaseCmd.linear.z=0;

    moveBaseCmd.angular.x=0;
    moveBaseCmd.angular.y=0;
    moveBaseCmd.angular.z=angularVelocity;
}

void RvizGui::moveBaseRight()
{
    ROS_INFO("move right");

    moveBaseCmd.linear.x=0;
    moveBaseCmd.linear.y=0;
    moveBaseCmd.linear.z=0;

    moveBaseCmd.angular.x=0;
    moveBaseCmd.angular.y=0;
    moveBaseCmd.angular.z=-angularVelocity;
}

void RvizGui::sendMoveBaseCmd()
{
    if(ros::ok() && moveBaseCmdPub)
    {
        moveBaseCmdPub.publish(moveBaseCmd);
        ROS_INFO("move base cmd sent");
    }
}
