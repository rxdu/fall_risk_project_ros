#ifndef FALLRISK_GUI_H
#define FALLRISK_GUI_H

#include <QMainWindow>

namespace Ui {
class FallRiskGUI;
}

class FallRiskGUI : public QMainWindow
{
    Q_OBJECT

public:
    explicit FallRiskGUI(QWidget *parent = 0);
    ~FallRiskGUI();

private:
    Ui::FallRiskGUI *ui;
};

#endif // FALLRISK_GUI_H
