#include "fallrisk_robot_gui.h"
#include "ui_fallrisk_robot_gui.h"
#include "rviz/render_panel.h"
#include "rviz/visualization_manager.h"

FallRiskRobotGUI::FallRiskRobotGUI(QWidget *parent) :
    QMainWindow(parent),it_(nh_),
    ui(new Ui::FallRiskRobotGUI)
{
    ui->setupUi(this);
    QWidget::showMaximized();

    imageTopic_ = QString("/camera/rgb/operator_image");
    liveVideoSub = it_.subscribe(imageTopic_.toStdString(),1,&FallRiskRobotGUI::liveVideoCallback,this,image_transport::TransportHints("compressed"));

    rviz::VisualizationManager* manager_ = new rviz::VisualizationManager( new rviz::RenderPanel() );
    manager_->initialize();
    manager_->startUpdate();
    }

FallRiskRobotGUI::~FallRiskRobotGUI()
{
    delete ui;
}

void FallRiskRobotGUI::liveVideoCallback(const sensor_msgs::ImageConstPtr& msg)
{

    cv_bridge::CvImagePtr cv_ptr, cv_ptr_big;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR16);
        cv_ptr_big = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR16);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //  convert cv image into RGB image and resize it to the size of available layout
    setVideo(ui->lbRobotMain,cv_ptr);
    setVideo(ui->lbRobotMain,cv_ptr_big);
}

void FallRiskRobotGUI::setVideo(QLabel* label, cv_bridge::CvImagePtr cv_ptr){
    cv::Mat RGBImg;
    QLabel* liveVideoLabel = label;

    int height = liveVideoLabel->height()-1;
    int width =  liveVideoLabel->width()-1;

    if(liveVideoLabel->height()-1 >= (liveVideoLabel->width()-1)*3/4)
        height= (liveVideoLabel->width()-1)*3/4;
    else
        width = (liveVideoLabel->height()-1)*4/3;

    cv::cvtColor(cv_ptr->image,RGBImg,CV_BGR2RGB);
    cv::resize(RGBImg,RGBImg,cvSize(width,height));

    //  convert RGB image into QImage and publish that on the label for livevideo
    QImage qImage_= QImage((uchar*) RGBImg.data, RGBImg.cols, RGBImg.rows, RGBImg.cols*3, QImage::Format_RGB888);
    liveVideoLabel->setPixmap(QPixmap::fromImage(qImage_));
    liveVideoLabel->show();

}
