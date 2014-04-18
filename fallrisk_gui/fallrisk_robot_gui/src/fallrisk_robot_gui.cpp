#include "fallrisk_robot_gui.h"
#include "ui_fallrisk_robot_gui.h"

FallRiskRobotGUI::FallRiskRobotGUI(QWidget *parent) :
    QMainWindow(parent),it_(nh_),
    ui(new Ui::FallRiskRobotGUI)
{
    ui->setupUi(this);

//    imageTopic_ = QString("/image_raw");
    baseSensorStatus = nh_.subscribe("/mobile_base/sensors/core",1,&FallRiskRobotGUI::baseStatusCheck,this);
//    liveVideoSub = it_.subscribe(imageTopic_.toStdString(),1,&FallRiskRobotGUI::liveVideoCallback,this,image_transport::TransportHints("compressed"));
    liveVideoSub = it_.subscribe("/image_raw",1,&FallRiskRobotGUI::liveVideoCallback,this);

    ROS_INFO("ROBOT STARTED!");
}

FallRiskRobotGUI::~FallRiskRobotGUI()
{
    delete ui;
}

void FallRiskRobotGUI::baseStatusCheck(const kobuki_msgs::SensorState::ConstPtr& msg)
{
    ROS_INFO("SENSOR DATA RECEIVED!");
    /*---------- battery of kobuki base -----------*/
    //    ROS_INFO("battery: %d",msg->battery);

    int battery_percentage = 0;

    battery_percentage = (msg->battery - BASE_BATTERY_DANGER)*100/(BASE_BATTERY_CAP-BASE_BATTERY_DANGER);
//    ui->pbBaseBattery->setValue(battery_percentage);
    //    if(msg->battery <= BASE_BATTERY_LOW)
    //    QPalette p = ui->pbBaseBattery->palette();
    //        p.setColor(QPalette::Highlight, Qt::red);
    //        ui->pbBaseBattery->setPalette(p);
    //    else
    //ui->pbBaseBattery->setStyleSheet(safe);

    /*-------------- bumper sensors ---------------*/
    if(msg->bumper == msg->BUMPER_LEFT)
    {
        ROS_INFO("BUMPER LEFT");
//        ui->lbBumperLeft->setStyleSheet("QLabel { background-color : rgb(255, 0, 0); color : rgb(255, 255, 255); }");
    }
    else if(msg->bumper == msg->BUMPER_CENTRE)
    {
        ROS_INFO("BUMPER CENTER");
//        ui->lbBumperCenter->setStyleSheet("QLabel { background-color : rgb(255, 0, 0); color : rgb(255, 255, 255); }");
    }
    else if(msg->bumper == msg->BUMPER_RIGHT)
    {
        ROS_INFO("BUMPER RIGHT");
//        ui->lbBumperRight->setStyleSheet("QLabel { background-color : rgb(255, 0, 0); color : rgb(255, 255, 255); }");
    }
    else
    {
//        ui->lbBumperCenter->setStyleSheet("QLabel { background-color : rgb(0, 204, 102); color : rgb(255, 255, 255); }");
//        ui->lbBumperLeft->setStyleSheet("QLabel { background-color : rgb(0, 204, 102); color : rgb(255, 255, 255); }");
//        ui->lbBumperRight->setStyleSheet("QLabel { background-color : rgb(0, 204, 102); color : rgb(255, 255, 255); }");
    }

    /*------------ wheel drop sensors -------------*/
    if(msg->wheel_drop == msg->WHEEL_DROP_LEFT)
    {
        ROS_INFO("wheel drop left");
//        ui->lbWheelLeft->setStyleSheet("QLabel { background-color : rgb(255, 0, 0); color : rgb(255, 255, 255); }");
    }
    else if(msg->wheel_drop == msg->WHEEL_DROP_RIGHT)
    {
        ROS_INFO("wheel drop right");
//        ui->lbWheelRight->setStyleSheet("QLabel { background-color : rgb(255, 0, 0); color : rgb(255, 255, 255); }");
    }
    else if(msg->wheel_drop == ( msg->WHEEL_DROP_LEFT+msg->WHEEL_DROP_RIGHT))
    {
        ROS_INFO("wheel drop both");
//        ui->lbWheelLeft->setStyleSheet("QLabel { background-color : rgb(255, 0, 0); color : rgb(255, 255, 255); }");
//        ui->lbWheelRight->setStyleSheet("QLabel { background-color : rgb(255, 0, 0); color : rgb(255, 255, 255); }");
    }
    else
    {
//        ui->lbWheelLeft->setStyleSheet("QLabel { background-color : rgb(0, 204, 102); color : rgb(255, 255, 255); }");
//        ui->lbWheelRight->setStyleSheet("QLabel { background-color : rgb(0, 204, 102); color : rgb(255, 255, 255); }");
    }

    /*-------------- cliff sensors ---------------*/
    if(msg->cliff == msg->CLIFF_LEFT)
    {
        ROS_INFO("cliff left");
//        ui->lbCliffLeft->setStyleSheet("QLabel { background-color : rgb(255, 0, 0); color : rgb(255, 255, 255); }");
    }
    else if(msg->cliff == msg->CLIFF_CENTRE)
    {
        ROS_INFO("cliff center");
//        ui->lbCliffCenter->setStyleSheet("QLabel { background-color : rgb(255, 0, 0); color : rgb(255, 255, 255); }");
    }
    else if(msg->cliff == msg->CLIFF_RIGHT)
    {
        ROS_INFO("cliff right");
//        ui->lbCliffRight->setStyleSheet("QLabel { background-color : rgb(255, 0, 0); color : rgb(255, 255, 255); }");
    }
    else
    {
//        ui->lbCliffCenter->setStyleSheet("QLabel { background-color : rgb(0, 204, 102); color : rgb(255, 255, 255); }");
//        ui->lbCliffLeft->setStyleSheet("QLabel { background-color : rgb(0, 204, 102); color : rgb(255, 255, 255); }");
//        ui->lbCliffRight->setStyleSheet("QLabel { background-color : rgb(0, 204, 102); color : rgb(255, 255, 255); }");
    }
}

void FallRiskRobotGUI::liveVideoCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO("IMAGE RECEIVED!");

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
