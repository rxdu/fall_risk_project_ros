#ifndef RVIZ_GUI_H
#define RVIZ_GUI_H

#include <QWidget>
#include <QEvent>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

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
    void moveForward();
    void moveBackward();
    void moveLeft();
    void moveRight();

private:
  rviz::VisualizationManager* manager_;
  rviz::RenderPanel* render_panel_;
  rviz::Display* grid_;

private:
  ros::NodeHandle nh_;
  ros::Publisher moveBaseCmdPub;
  geometry_msgs::Twist moveBaseCmd;

};

#endif // RVIZ_GUI_H
