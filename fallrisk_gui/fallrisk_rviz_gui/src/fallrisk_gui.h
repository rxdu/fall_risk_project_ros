#ifndef FALLRISK_GUI_H
#define FALLRISK_GUI_H

#include <QMainWindow>
#include <QWidget>
#include <QEvent>
#include <QKeyEvent>
#include <QStatusBar>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/panel.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define LIN_VEL_MAX 0.25
#define LIN_VEL_MIN 0.08
#define ANG_VEL_MAX 1.2
#define ANG_VEL_MIN 0.65

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
  rviz::RenderPanel* render_panel_;
  rviz::Display* mainDisplay_;
  rviz::Display* imageDisplay_;
  rviz::Panel* imagePanel_;

private:
  ros::NodeHandle nh_;
  ros::Publisher moveBaseCmdPub;
  geometry_msgs::Twist moveBaseCmd;
  float linearVelocity;
  float angularVelocity;
};

#endif // FALLRISK_GUI_H
