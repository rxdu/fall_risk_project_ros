#include "rviz_gui.h"
#include "ui_rviz_gui.h"

RvizGui::RvizGui(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::RvizGui)
{
    ui->setupUi(this);

    // Initialize GUI elements
    render_panel_ = new rviz::RenderPanel();
    ui->display3d_layout->addWidget(render_panel_);

    //image_display_ = new rviz::ImageDisplay();
    //ui->livevideo_layout->addWidget(image_display_);

    manager_ = new rviz::VisualizationManager( render_panel_ );
    render_panel_->initialize( manager_->getSceneManager(), manager_ );
    manager_->initialize();
    manager_->startUpdate();

    // Create a Grid display.
    grid_ = manager_->createDisplay( "rviz/Grid", "adjustable grid", true );
    ROS_ASSERT( grid_ != NULL );

    // Configure the GridDisplay the way we like it.
    grid_->subProp( "Line Style" )->setValue( "Billboards" );
    grid_->subProp( "Color" )->setValue( Qt::yellow );

}

RvizGui::~RvizGui()
{
    delete ui;
}
