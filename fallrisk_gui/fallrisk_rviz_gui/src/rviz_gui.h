#ifndef RVIZ_GUI_H
#define RVIZ_GUI_H

#include <QWidget>
#include <QEvent>
#include <QKeyEvent>
#include <QStatusBar>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define LIN_VEL_MAX 0.25
#define LIN_VEL_MIN 0.08
#define ANG_VEL_MAX 1.2
#define ANG_VEL_MIN 0.65

namespace Ui {
class RvizGui;
}

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
class ImageDisplay;
}

class RvizGui : public QWidget
{
    Q_OBJECT

public:
    explicit RvizGui(QWidget *parent = 0);
    ~RvizGui();

private:
    Ui::RvizGui *ui;

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

private:
  ros::NodeHandle nh_;
  ros::Publisher moveBaseCmdPub;
  geometry_msgs::Twist moveBaseCmd;
  float linearVelocity;
  float angularVelocity;

};

#endif // RVIZ_GUI_H
