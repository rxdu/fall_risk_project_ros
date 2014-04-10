#ifndef FALLRISK_GUI_H
#define FALLRISK_GUI_H

#include <QMainWindow>
#include <QWidget>
#include <QEvent>
#include <QKeyEvent>
#include <QStatusBar>
#include <QImage>
#include <QPainter>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/panel.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Image.h>
#include <kobuki_msgs/SensorState.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#define LIN_VEL_MAX 0.25
#define LIN_VEL_MIN 0.08
#define ANG_VEL_MAX 1.2
#define ANG_VEL_MIN 0.65

#define BASE_BATTERY_CAP 165
#define BASE_BATTERY_LOW 140
#define BASE_BATTERY_DANGER 132

namespace Ui {
class FallRiskGUI;
}

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
class ImageDisplay;
class Panel;
}

class FallRiskGUI : public QMainWindow
{
    Q_OBJECT

public:
    explicit FallRiskGUI(QWidget *parent = 0);
    ~FallRiskGUI();


private:
    Ui::FallRiskGUI *ui;

private:
    void initActionsConnections();
    void initDisplayWidgets();
    void initVariables();

    void sendMoveBaseCmd();


private Q_SLOTS:
    void moveBaseForward();
    void moveBaseBackward();
    void moveBaseLeft();
    void moveBaseRight();
    void keyPressEvent(QKeyEvent *event);
    void setRobotVelocity();

private:
  rviz::VisualizationManager* manager_;
  rviz::VisualizationManager* mapManager_;
  rviz::RenderPanel* renderPanel_;
  rviz::RenderPanel* mapRenderPanel_ ;
  rviz::ViewManager* viewManager_;
  rviz::ViewController* viewController_ ;
  rviz::Display* mainDisplay_;
//  rviz::Display* imageDisplay_;
  rviz::Display* octomapDisplay_;
  rviz::Display* mapDisplay_ ;
  rviz::RenderPanel* imagePanel_;

private:
  ros::NodeHandle nh_;
  ros::Publisher moveBaseCmdPub;
  ros::Subscriber centerDistSub;
  ros::Subscriber baseSensorStatus;

  image_transport::ImageTransport it_;
  image_transport::Subscriber liveVideoSub;

  geometry_msgs::Twist moveBaseCmd;
  float linearVelocity;
  float angularVelocity;

  void distanceSubCallback(const std_msgs::Float32::ConstPtr& msg);
  void baseStatusCheck(const kobuki_msgs::SensorState::ConstPtr& msg);
  void liveVideoCallback(const sensor_msgs::ImageConstPtr &msg);


  QString fixedFrame_;
  QString mapTopic_;
  QString imageTopic_;
  QString pointCloudTopic_;
  QString octomapTopic_;
  QString baseSensorTopic_;
  QString velocityTopic_;

};


#endif // FALLRISK_GUI_H
