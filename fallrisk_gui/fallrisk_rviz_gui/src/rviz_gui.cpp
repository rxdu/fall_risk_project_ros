#include "rviz_gui.h"
#include "ui_rviz_gui.h"

RvizGui::RvizGui(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::RvizGui)
{
    ui->setupUi(this);
}

RvizGui::~RvizGui()
{
    delete ui;
}
