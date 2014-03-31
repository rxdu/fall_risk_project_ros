#include "fallrisk_gui.h"
#include "ui_fallrisk_gui.h"

FallRiskGUI::FallRiskGUI(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::FallRiskGUI)
{
    ui->setupUi(this);
}

FallRiskGUI::~FallRiskGUI()
{
    delete ui;
}
