#ifndef RVIZ_GUI_H
#define RVIZ_GUI_H

#include <QWidget>

namespace Ui {
class RvizGui;
}

class RvizGui : public QWidget
{
    Q_OBJECT

public:
    explicit RvizGui(QWidget *parent = 0);
    ~RvizGui();

private:
    Ui::RvizGui *ui;
};

#endif // RVIZ_GUI_H
