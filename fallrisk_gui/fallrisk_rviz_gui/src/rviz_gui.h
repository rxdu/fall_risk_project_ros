#ifndef RVIZ_GUI_H
#define RVIZ_GUI_H

#include <QWidget>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"

namespace Ui {
class RvizGui;
}

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
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
  rviz::VisualizationManager* manager_;
  rviz::RenderPanel* render_panel_;
  rviz::Display* grid_;
};

#endif // RVIZ_GUI_H
