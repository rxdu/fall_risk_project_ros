#include "rviz_gui.h"
#include "ui_rviz_gui.h"
#include <sensor_msgs/Image.h>

RvizGui::RvizGui(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::RvizGui)
{
    ui->setupUi(this);

    // Initialize GUI elements
    render_panel_ = new rviz::RenderPanel();
    ui->display3d_layout->addWidget(render_panel_);

    manager_ = new rviz::VisualizationManager( render_panel_ );
    render_panel_->initialize( manager_->getSceneManager(), manager_ );
    manager_->initialize();
    manager_->startUpdate();

    grid_ = manager_->createDisplay( "rviz/Image", "image display grid", true );
    ROS_ASSERT( grid_ != NULL );
    // Create a Grid display.
    //   grid_ = manager_->createDisplay( "rviz/Grid", "adjustable grid", true );
    //   ROS_ASSERT( grid_ != NULL );

    // Configure the GridDisplay the way we like it.
    // grid_->subProp( "Line Style" )->setValue( "Billboards" );
    // grid_->subProp( "Color" )->setValue( Qt::yellow );

    grid_->setTopic("/image_raw","sensor_msgs/Image");


}

RvizGui::~RvizGui()
{
    delete ui;
}
