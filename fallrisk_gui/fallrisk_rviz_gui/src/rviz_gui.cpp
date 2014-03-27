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
}

void RvizGui::initActionsConnections()
{
    connect(ui->btnUp, SIGNAL(clicked()), this, SLOT(moveForward()));
    connect(ui->btnDown, SIGNAL(clicked()), this, SLOT(moveBackward()));
    connect(ui->btnLeft, SIGNAL(clicked()), this, SLOT(moveLeft()));
    connect(ui->btnRight, SIGNAL(clicked()), this, SLOT(moveRight()));
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

void RvizGui::moveForward()
{
    ROS_INFO("move forward");

    moveBaseCmd.linear.x=0.2;
    moveBaseCmd.linear.y=0;
    moveBaseCmd.linear.z=0;

    moveBaseCmd.angular.x=0;
    moveBaseCmd.angular.y=0;
    moveBaseCmd.angular.z=0;

    sendMoveBaseCmd();
}

void RvizGui::moveBackward()
{
    ROS_INFO("move backward");

    moveBaseCmd.linear.x=-0.2;
    moveBaseCmd.linear.y=0;
    moveBaseCmd.linear.z=0;

    moveBaseCmd.angular.x=0;
    moveBaseCmd.angular.y=0;
    moveBaseCmd.angular.z=0;

    sendMoveBaseCmd();
}

void RvizGui::moveLeft()
{
    ROS_INFO("move left");

    moveBaseCmd.linear.x=0;
    moveBaseCmd.linear.y=0;
    moveBaseCmd.linear.z=0;

    moveBaseCmd.angular.x=0;
    moveBaseCmd.angular.y=0;
    moveBaseCmd.angular.z=1;

    sendMoveBaseCmd();
}

void RvizGui::moveRight()
{
    ROS_INFO("move right");

    moveBaseCmd.linear.x=0;
    moveBaseCmd.linear.y=0;
    moveBaseCmd.linear.z=0;

    moveBaseCmd.angular.x=0;
    moveBaseCmd.angular.y=0;
    moveBaseCmd.angular.z=-1;

    sendMoveBaseCmd();
}

void RvizGui::sendMoveBaseCmd()
{
    if(ros::ok() && moveBaseCmdPub)
    {
        moveBaseCmdPub.publish(moveBaseCmd);
        ROS_INFO("move cmd sent");
    }
}
