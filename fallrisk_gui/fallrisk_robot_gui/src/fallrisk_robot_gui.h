#ifndef FALLRISK_ROBOT_GUI_H
#define FALLRISK_ROBOT_GUI_H

#include <QMainWindow>
#include <QLabel>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <kobuki_msgs/SensorState.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

namespace Ui {
class FallRiskRobotGUI;
}

class FallRiskRobotGUI : public QMainWindow
{
    Q_OBJECT

public:
    explicit FallRiskRobotGUI(QWidget *parent = 0);
    ~FallRiskRobotGUI();

private:
    Ui::FallRiskRobotGUI *ui;

private:
  ros::NodeHandle nh_;
  ros::Publisher baseSensorStatus;

  image_transport::ImageTransport it_;
  image_transport::Subscriber liveVideoSub;

  QString imageTopic_;

  void liveVideoCallback(const sensor_msgs::ImageConstPtr &msg);
  void setVideo(QLabel* label, cv_bridge::CvImagePtr cv_ptr);

};

#endif // FALLRISK_ROBOT_GUI_H
