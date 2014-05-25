#include "fallrisk_gui.h"
#include "ui_fallrisk_gui.h"
#include <iostream>
#include "rviz/view_manager.h"
#include "rviz/tool_manager.h"
#include "rviz/properties/property_tree_model.h"
#include "rviz/frame_manager.h"

/**
 * This class creates the GUI using rviz APIs.
 */

FallRiskGUI::FallRiskGUI(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::FallRiskGUI),it_(nh_)
{
    /**
     * Set up the QT related UI components.
     */
    ui->setupUi(this);
    ui->sliderLinearVel->setValue(75);
    ui->sliderAngularVel->setValue(75);
    ui->lbLightingItem1->setStyleSheet("QLabel { background-color : red; color : rgb(255, 255, 255); }");
    ui->lbLightingItem2->setStyleSheet("QLabel { background-color : green; color : rgb(255, 255, 255); }");
    ui->lbFloorsItem1->setStyleSheet("QLabel { background-color : green; color : rgb(255, 255, 255); }");
    ui->lbFloorsItem2->setStyleSheet("QLabel { background-color : green; color : rgb(255, 255, 255); }");
    ui->lbStairsItem1->setStyleSheet("QLabdel { background-color : green; color : rgb(255, 255, 255); }");
    ui->lbStairsItem2->setStyleSheet("QLabel { background-color : green; color : rgb(255, 255, 255); }");
    ui->lbStairsItem3->setStyleSheet("QLabel { background-color : red; color : rgb(255, 255, 255); }");
    ui->lbBedroomItem1->setStyleSheet("QLabel { background-color : green; color : rgb(255, 255, 255); }");
    ui->lbBedroomItem2->setStyleSheet("QLabel { background-color : red; color : rgb(255, 255, 255); }");
    ui->lbBedroomItem3->setStyleSheet("QLabel { background-color : green; color : rgb(255, 255, 255); }");
    ui->lbPetItem1->setStyleSheet("QLabel { background-color : yellow; color : rgb(255, 255, 255); }");
    changeToolBtnStatus(-2); //set the initial rviz tool to be "interact"
    ui->rbNavMode->setChecked(true);

    initVariables();
    initDisplayWidgets();
    initTools();
    initActionsConnections();

    setRobotNavMode(NAVIGATION_MODE);
}


FallRiskGUI::~FallRiskGUI()
{
    delete ui;
    delete mapManager_;
    delete mapRenderPanel_;
    delete manager_;
    delete renderPanel_;
    delete status_label_;
}

void FallRiskGUI::initVariables()
{
    /**
     *Initialize default values of all the variables. Push these definitions to xml/config file in future
     */
    baseFrame_ =  QString("/base_link");
    targetFrame_ =  QString("/camera_rgb_optical_frame");
    mapTopic_ = QString("/map");
    imageTopic_ = QString("/camera/rgb/image_raw");
    pointCloudTopic_=QString("/camera/depth/points");
    octomapTopic_=QString( "/occupied_cells_vis_array" );
    baseSensorTopic_=QString("/mobile_base/sensors/core");
    velocityTopic_=QString("/mobile_base/commands/velocity");
    pathTopic_ = QString("/move_base/NavfnROS/plan");

    moveBaseCmdPub = nh_.advertise<geometry_msgs::Twist>(velocityTopic_.toStdString(),1);
    centerDistSub = nh_.subscribe("/distance/image_center_dist",1,&FallRiskGUI::distanceSubCallback,this);
    baseSensorStatus = nh_.subscribe(baseSensorTopic_.toStdString(),1,&FallRiskGUI::baseStatusCheck,this);    
    lightingClient = nh_.serviceClient<checklist_status::ChecklistStatusSrv>("checklist_status");
    remoteCmdClient = nh_.serviceClient<remote_command_server::RemoteCmdSrv>("remote_command");

    liveVideoSub = it_.subscribe(imageTopic_.toStdString(),1,&FallRiskGUI::liveVideoCallback,this,image_transport::TransportHints("compressed"));

    setRobotVelocity();
}

void FallRiskGUI::initActionsConnections()
{
    /**
     * Set up the status Bar and display messages emitted from each of the tools.
     * All the tools in rviz API has updateStatus function to emit messages to the status bar.
     */
    status_label_ = new QLabel("");
    statusBar()->addPermanentWidget( status_label_,1);
    connect( manager_, SIGNAL( statusUpdate( const QString& )), status_label_, SLOT( setText( const QString& )));

    /**
     * Setup Signals and slots for different buttons/sliders in UI.
     */
    connect(ui->btnUp, SIGNAL(clicked()), this, SLOT(moveBaseForward()));
    connect(ui->btnDown, SIGNAL(clicked()), this, SLOT(moveBaseBackward()));
    connect(ui->btnLeft, SIGNAL(clicked()), this, SLOT(moveBaseLeft()));
    connect(ui->btnRight, SIGNAL(clicked()), this, SLOT(moveBaseRight()));

    connect(ui->btnGroupRvizTools,SIGNAL(buttonClicked(int)),this,SLOT(setCurrentTool(int)));
    connect(ui->btnGroupNavModeSwitch,SIGNAL(buttonClicked(int)),this,SLOT(setRobotNavMode(int)));
//    connect(ui->rbNavMode,SIGNAL(clicked()),this,SLOT(setRobotToAmclMode()));
//    connect(ui->rbNavMode,SIGNAL(clicked()),this,SLOT(setRobotToGmappingMode()));

    connect(ui->sliderLinearVel, SIGNAL(valueChanged(int)),this,SLOT(setRobotVelocity()));
    connect(ui->sliderAngularVel, SIGNAL(valueChanged(int)),this,SLOT(setRobotVelocity()));

    connect(ui->tab_display, SIGNAL(currentChanged(int)),this,SLOT(setActiveRvizToolBtns(int)));
}

void FallRiskGUI::initDisplayWidgets()
{

    //Setup the UI elements for displaying 2D map
    /**
     * VisualizationManager is used to control different displays that are shown in a widget.
     * Renderpanel is a widget that provides a 3D space in the visualizationmanager.
     * startUpdate() function starts the timers and subscribes to defined topics at 30Hz.
     */
    mapRenderPanel_ = new rviz::RenderPanel();
    ui->map_layout->addWidget(mapRenderPanel_);
    mapManager_ = new rviz::VisualizationManager( mapRenderPanel_ );
    mapRenderPanel_->initialize( mapManager_->getSceneManager(), mapManager_);
    mapManager_->setFixedFrame(mapTopic_);
    mapManager_->initialize();
    mapManager_->startUpdate();
    mapManager_->initialize();

    //Create and assign FixedOrientationOrthoViewController to the existing viewmanager of the visualization manager
    /**
     * VisualisationManager has a manager for most of its children. ViewManager is responsible for setting the viewController.
     * Default View Controller is rviz/Orbit, for map we are changing it to rviz/TopDownOrtho
     * To set properties of most of the rviz objects, use subProp and setValue functions as shown below
     * New displays can be created and added to the visualization manager using createDisplay function as used below
     *
     * @todo Create an xml/config file to define objects to be displayed in GUI alongwith their parameters
     */
    mapViewManager_ = mapManager_->getViewManager();
    mapViewManager_->setCurrentViewControllerType("rviz/TopDownOrtho");
    mapViewController_ = mapViewManager_->getCurrent();

    //Set parameters of the view controller to show map correctly
    mapViewController_->subProp("X")->setValue(0);
    mapViewController_->subProp("Y")->setValue(0);
    mapViewController_->subProp("Angle")->setValue(0);
    mapViewController_->subProp("Scale")->setValue(20);

    // Create a map display
    mapDisplay_ = mapManager_->createDisplay( "rviz/Map", "2D Map view", true );
    ROS_ASSERT( mapDisplay_ != NULL );

    mapDisplay_->subProp( "Topic" )->setValue( mapTopic_ );

    mapManager_->createDisplay( "rviz/RobotModel", "Turtlebot", true );

    mapManager_->createDisplay("rviz/Path","Global path",true)->subProp( "Topic" )->setValue(pathTopic_);


    // Initialize GUI elements for main panel
    renderPanel_ = new rviz::RenderPanel();
    ui->display3d_layout->addWidget(renderPanel_);

    manager_ = new rviz::VisualizationManager( renderPanel_ );
    renderPanel_->initialize( manager_->getSceneManager(), manager_ );

    //set the fixed frame before initializing Visualization Manager. pointcloud2 will not work with this
    manager_->setFixedFrame(mapTopic_);
    manager_->initialize();
    manager_->startUpdate();


    // Create a main display to show pointcloud and octomap

    manager_->createDisplay( "rviz/Grid", "Grid", true );
    manager_->createDisplay( "rviz/RobotModel", "Turtlebot", true );

    octomapDisplay_ = manager_->createDisplay( "rviz/MarkerArray", "Octomap view", true );
    ROS_ASSERT( octomapDisplay_ != NULL );

    octomapDisplay_->subProp( "Marker Topic" )->setValue(octomapTopic_);

    //Assign Target Frame to the existing viewmanager of the visualization manager
    rviz::ViewManager* viewManager_ = manager_->getViewManager();
    rviz::ViewController* viewController_ = viewManager_->getCurrent();
    viewController_->subProp("Target Frame")->setValue(targetFrame_);
    manager_->createDisplay("rviz/Path","Global path",true)->subProp( "Topic" )->setValue(pathTopic_);

}

void FallRiskGUI::initTools(){
    /**
     * ToolManager is similar to ViewManager. It can be used to add new tools and change the current or default tool.
     * Properties of tools are stored in a PropertyTreeModel. To set/modify any property of a tool use getPropertyContainer function.
     */
    toolManager_ = manager_->getToolManager();

    pointTool_ = toolManager_->addTool("rviz/PublishPoint");
    measureTool_ = toolManager_->addTool("rviz/Measure");
    setGoalTool_ = toolManager_->addTool("rviz/SetGoal");
    setInitialPoseTool_=toolManager_->addTool("rviz/SetInitialPose");
    interactTool_ = toolManager_->addTool("rviz/Interact");

    mapToolManager_ = mapManager_->getToolManager();

    mapInteractTool_ = mapToolManager_->addTool("rviz/Interact");
    setMapGoalTool_ = mapToolManager_->addTool("rviz/SetGoal");
    setMapInitialPoseTool_ = mapToolManager_->addTool("rviz/SetInitialPose");

    // Find the entry in propertytreemodel and set the value for Topic
    setGoalTool_->getPropertyContainer()->subProp("Topic")->setValue("/move_base_simple/goal");
    setMapGoalTool_->getPropertyContainer()->subProp("Topic")->setValue("/move_base_simple/goal");

}

void FallRiskGUI::keyPressEvent(QKeyEvent *event)
{
    switch(event->key())
    {
    case Qt::Key_W:
        moveBaseForward();
        ROS_INFO("key W pressed");
        break;
    case Qt::Key_A:
        moveBaseLeft();
        ROS_INFO("key A pressed");
        break;
    case Qt::Key_D:
        moveBaseRight();
        ROS_INFO("key D pressed");
        break;
    case Qt::Key_S:
        moveBaseBackward();
        ROS_INFO("key S pressed");
        break;
    default:
        QWidget::keyPressEvent(event);
        break;
    }
}

void FallRiskGUI::distanceSubCallback(const std_msgs::Float32::ConstPtr& msg)
{
    QLocale english(QLocale::English, QLocale::UnitedStates);
    QString qdist = english.toString(msg->data, 'f', 2);
    ui->lbDistance->setText(qdist);
}

void FallRiskGUI::baseStatusCheck(const kobuki_msgs::SensorState::ConstPtr& msg)
{
    /*---------- battery of kobuki base -----------*/
    //    ROS_INFO("battery: %d",msg->battery);

    int battery_percentage = 0;

    battery_percentage = (msg->battery - BASE_BATTERY_DANGER)*100/(BASE_BATTERY_CAP-BASE_BATTERY_DANGER);
    ui->pbBaseBattery->setValue(battery_percentage);
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
        ui->lbBumperLeft->setStyleSheet("QLabel { background-color : rgb(255, 0, 0); color : rgb(255, 255, 255); }");
    }
    else if(msg->bumper == msg->BUMPER_CENTRE)
    {
        ROS_INFO("BUMPER CENTER");
        ui->lbBumperCenter->setStyleSheet("QLabel { background-color : rgb(255, 0, 0); color : rgb(255, 255, 255); }");
    }
    else if(msg->bumper == msg->BUMPER_RIGHT)
    {
        ROS_INFO("BUMPER RIGHT");
        ui->lbBumperRight->setStyleSheet("QLabel { background-color : rgb(255, 0, 0); color : rgb(255, 255, 255); }");
    }
    else
    {
        ui->lbBumperCenter->setStyleSheet("QLabel { background-color : rgb(0, 204, 102); color : rgb(255, 255, 255); }");
        ui->lbBumperLeft->setStyleSheet("QLabel { background-color : rgb(0, 204, 102); color : rgb(255, 255, 255); }");
        ui->lbBumperRight->setStyleSheet("QLabel { background-color : rgb(0, 204, 102); color : rgb(255, 255, 255); }");
    }

    /*------------ wheel drop sensors -------------*/
    if(msg->wheel_drop == msg->WHEEL_DROP_LEFT)
    {
        ROS_INFO("wheel drop left");
        ui->lbWheelLeft->setStyleSheet("QLabel { background-color : rgb(255, 0, 0); color : rgb(255, 255, 255); }");
    }
    else if(msg->wheel_drop == msg->WHEEL_DROP_RIGHT)
    {
        ROS_INFO("wheel drop right");
        ui->lbWheelRight->setStyleSheet("QLabel { background-color : rgb(255, 0, 0); color : rgb(255, 255, 255); }");
    }
    else if(msg->wheel_drop == ( msg->WHEEL_DROP_LEFT+msg->WHEEL_DROP_RIGHT))
    {
        ROS_INFO("wheel drop both");
        ui->lbWheelLeft->setStyleSheet("QLabel { background-color : rgb(255, 0, 0); color : rgb(255, 255, 255); }");
        ui->lbWheelRight->setStyleSheet("QLabel { background-color : rgb(255, 0, 0); color : rgb(255, 255, 255); }");
    }
    else
    {
        ui->lbWheelLeft->setStyleSheet("QLabel { background-color : rgb(0, 204, 102); color : rgb(255, 255, 255); }");
        ui->lbWheelRight->setStyleSheet("QLabel { background-color : rgb(0, 204, 102); color : rgb(255, 255, 255); }");
    }

    /*-------------- cliff sensors ---------------*/
    if(msg->cliff == msg->CLIFF_LEFT)
    {
        ROS_INFO("cliff left");
        ui->lbCliffLeft->setStyleSheet("QLabel { background-color : rgb(255, 0, 0); color : rgb(255, 255, 255); }");
    }
    else if(msg->cliff == msg->CLIFF_CENTRE)
    {
        ROS_INFO("cliff center");
        ui->lbCliffCenter->setStyleSheet("QLabel { background-color : rgb(255, 0, 0); color : rgb(255, 255, 255); }");
    }
    else if(msg->cliff == msg->CLIFF_RIGHT)
    {
        ROS_INFO("cliff right");
        ui->lbCliffRight->setStyleSheet("QLabel { background-color : rgb(255, 0, 0); color : rgb(255, 255, 255); }");
    }
    else
    {
        ui->lbCliffCenter->setStyleSheet("QLabel { background-color : rgb(0, 204, 102); color : rgb(255, 255, 255); }");
        ui->lbCliffLeft->setStyleSheet("QLabel { background-color : rgb(0, 204, 102); color : rgb(255, 255, 255); }");
        ui->lbCliffRight->setStyleSheet("QLabel { background-color : rgb(0, 204, 102); color : rgb(255, 255, 255); }");
    }

    /*----------- illuminosity sensor ------------*/
    if(lightingClient.call(lightingSrv))
    {
        ROS_INFO("Luminosity: %f", lightingSrv.response.luminosity);
        if(lightingSrv.response.luminosity>=200.00)
        {
            setChecklistItemColor(ui->lbLightingItem1, CHECKLIST_ITEM_GREEN);
            setChecklistItemStatus(ui->cbLightingItem1, CHECKLIST_ITEM_CHECKED);
        }
        else if(lightingSrv.response.luminosity>=100.00)
        {
            setChecklistItemColor(ui->lbLightingItem1, CHECKLIST_ITEM_YELLOW);
            setChecklistItemStatus(ui->cbLightingItem1, CHECKLIST_ITEM_UNCHECKED);
        }
        else
        {
            setChecklistItemColor(ui->lbLightingItem1, CHECKLIST_ITEM_RED);
            setChecklistItemStatus(ui->cbLightingItem1, CHECKLIST_ITEM_UNCHECKED);
        }
    }
    else
    {
        ROS_ERROR("Failed to call service checklist_status");
    }
}

void FallRiskGUI::liveVideoCallback(const sensor_msgs::ImageConstPtr& msg)
{

    /**
     * Adding Image display opens up the image in a new window.
     * As a workaround to show image in the same GUI window, OpenCV is being used to display image on a Qlabel
     */
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
    setVideo(ui->liveVideoLabel,cv_ptr);
    setVideo(ui->lbLiveVideoBig,cv_ptr_big);
}

void FallRiskGUI::setVideo(QLabel* label, cv_bridge::CvImagePtr cv_ptr){
    cv::Mat RGBImg;
    QLabel* liveVideoLabel = label;

    // To avoid auto expansion of QLabel,keep the video dimensions slightly less than the label dimension
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

void FallRiskGUI::setRobotVelocity()
{
    linearVelocity = ui->sliderLinearVel->value()*(LIN_VEL_MAX-LIN_VEL_MIN)/100+LIN_VEL_MIN;
    ROS_INFO("Linear velocity:%f",linearVelocity);
    angularVelocity = ui->sliderAngularVel->value()*(ANG_VEL_MAX-ANG_VEL_MIN)/100+ANG_VEL_MIN;
    ROS_INFO("Angular velocity:%f",angularVelocity);
}

void FallRiskGUI::moveBaseForward()
{
    ROS_INFO("move forward");

    moveBaseCmd.linear.x=linearVelocity;
    moveBaseCmd.linear.y=0;
    moveBaseCmd.linear.z=0;

    moveBaseCmd.angular.x=0;
    moveBaseCmd.angular.y=0;
    moveBaseCmd.angular.z=0;

    sendMoveBaseCmd();
}

void FallRiskGUI::moveBaseBackward()
{
    ROS_INFO("move backward");

    moveBaseCmd.linear.x=-linearVelocity;
    moveBaseCmd.linear.y=0;
    moveBaseCmd.linear.z=0;

    moveBaseCmd.angular.x=0;
    moveBaseCmd.angular.y=0;
    moveBaseCmd.angular.z=0;

    sendMoveBaseCmd();
}

void FallRiskGUI::moveBaseLeft()
{
    ROS_INFO("move left");

    moveBaseCmd.linear.x=0;
    moveBaseCmd.linear.y=0;
    moveBaseCmd.linear.z=0;

    moveBaseCmd.angular.x=0;
    moveBaseCmd.angular.y=0;
    moveBaseCmd.angular.z=angularVelocity;

    sendMoveBaseCmd();
}

void FallRiskGUI::moveBaseRight()
{
    ROS_INFO("move right");

    moveBaseCmd.linear.x=0;
    moveBaseCmd.linear.y=0;
    moveBaseCmd.linear.z=0;

    moveBaseCmd.angular.x=0;
    moveBaseCmd.angular.y=0;
    moveBaseCmd.angular.z=-angularVelocity;

    sendMoveBaseCmd();
}

void FallRiskGUI::sendMoveBaseCmd()
{
    if(ros::ok() && moveBaseCmdPub)
    {
        moveBaseCmdPub.publish(moveBaseCmd);
        ROS_INFO("move base cmd sent");
    }
}

void FallRiskGUI::setCurrentTool(int btnID)
{
    if(btnID == -2)
    {
        ROS_INFO("Interact Tool Selected");
        toolManager_->setCurrentTool(interactTool_);        
        mapToolManager_->setCurrentTool(mapInteractTool_);

    }
    else if(btnID == -3)
    {
        ROS_INFO("Measure Tool Selected");
        toolManager_->setCurrentTool(measureTool_);

    }
    else if(btnID == -4)
    {
        ROS_INFO("2DPoseEstimate Tool Selected");
        toolManager_->setCurrentTool(setInitialPoseTool_);
        mapToolManager_->setCurrentTool(setMapInitialPoseTool_);
    }
    else if(btnID == -5)
    {
        ROS_INFO("2DNavGoal Tool Selected");
        toolManager_->setCurrentTool(setGoalTool_);
        mapManager_->getToolManager()->setCurrentTool(setMapGoalTool_);
    }
    else if(btnID == -6)
    {
        ROS_INFO("PublishPoint Tool Selected");
        toolManager_->setCurrentTool(pointTool_);
    }

    changeToolBtnStatus(btnID);
}

void FallRiskGUI::changeToolBtnStatus(int btnID)
{
    ui->btnRvizInteract->setFlat(true);
    ui->btnRvizMeasure->setFlat(true);
    ui->btnRvizNavGoal->setFlat(true);
    ui->btnRvizPoseEstimate->setFlat(true);
    ui->btnRvizPublishPoint->setFlat(true);

    switch(btnID)
    {
    case -2: ui->btnRvizInteract->setFlat(false);
        break;
    case -3: ui->btnRvizMeasure->setFlat(false);
        break;
    case -4: ui->btnRvizPoseEstimate->setFlat(false);
        break;
    case -5: ui->btnRvizNavGoal->setFlat(false);
        break;
    case -6: ui->btnRvizPublishPoint->setFlat(false);
    }
}

void FallRiskGUI::setActiveRvizToolBtns(int tabID)
{
//    ROS_INFO("TAB:%d",tabID);

    ui->btnRvizInteract->setDisabled(false);
    ui->btnRvizMeasure->setDisabled(false);
    ui->btnRvizPoseEstimate->setDisabled(false);
    ui->btnRvizNavGoal->setDisabled(false);
    ui->btnRvizPublishPoint->setDisabled(false);

    if(tabID == 1)
    {
        ui->btnRvizMeasure->setDisabled(true);
        ui->btnRvizPublishPoint->setDisabled(true);
    }
    else if(tabID == 2)
    {
        ui->btnRvizInteract->setDisabled(true);
        ui->btnRvizMeasure->setDisabled(true);
        ui->btnRvizPoseEstimate->setDisabled(true);
        ui->btnRvizNavGoal->setDisabled(true);
        ui->btnRvizPublishPoint->setDisabled(true);
    }
}

void FallRiskGUI::setChecklistItemColor(QLabel *label, int color)
{
    //status: 0-green,1-yellow,2-red
    switch(color)
    {
    case CHECKLIST_ITEM_GREEN:
        label->setStyleSheet("QLabel { background-color : green; color : rgb(255, 255, 255); }");
        break;
    case CHECKLIST_ITEM_YELLOW:
        label->setStyleSheet("QLabel { background-color : yellow; color : rgb(255, 255, 255); }");
        break;
    case CHECKLIST_ITEM_RED:
        label->setStyleSheet("QLabel { background-color : red; color : rgb(255, 255, 255); }");
        break;
    }
}

void FallRiskGUI::setChecklistItemStatus(QCheckBox *checkbox, int status)
{
    switch(status)
    {
    case CHECKLIST_ITEM_CHECKED:
        checkbox->setChecked(true);
        break;
    case CHECKLIST_ITEM_UNCHECKED:
        checkbox->setChecked(false);
        break;
    }
}

void FallRiskGUI::setRobotNavMode(int modeID)
{
//    ROS_INFO("ID:%d",modeID);

    if(modeID == NAVIGATION_MODE) //navigation mode
    {
        remote_command_server::RemoteCmdSrv remoteCmdSrv;

        remoteCmdSrv.request.cmd_name=remoteCmdSrv.request.CMD_AMCL;
        remoteCmdSrv.request.cmd_action=remoteCmdSrv.request.START;

        if(remoteCmdClient.call(remoteCmdSrv))
        {
            if(remoteCmdSrv.response.cmd_status)
                ROS_INFO("AMCL successfully started");
            else
                ROS_INFO("AMCL failed to get started");
//            this->setFixedFrame(mapTopic_);
        }
        else
        {
            ROS_ERROR("Failed to call service remote_command");
        }
    }
    else if(modeID == MAPPING_MODE) //mapping mode
    {
        remote_command_server::RemoteCmdSrv remoteCmdSrv;

        remoteCmdSrv.request.cmd_name=remoteCmdSrv.request.CMD_GMAPPING;
        remoteCmdSrv.request.cmd_action=remoteCmdSrv.request.START;

        if(remoteCmdClient.call(remoteCmdSrv))
        {
            if(remoteCmdSrv.response.cmd_status)
                ROS_INFO("GMAPPING successfully started");
            else
                ROS_INFO("GMAPPING failed to get started");
//            this->setFixedFrame(baseFrame_);
        }
        else
        {
            ROS_ERROR("Failed to call service remote_command");
        }
    }

}

void FallRiskGUI::setFixedFrame(const QString fixedFrame){
    mapManager_->getFrameManager();
    mapManager_->setFixedFrame(fixedFrame);
    mapManager_->initialize();
    mapManager_->startUpdate();
    std::cout<<mapManager_->getFixedFrame().toStdString()<<std::endl;
    manager_->setFixedFrame(fixedFrame);
    manager_->initialize();
    manager_->startUpdate();
    std::cout<<manager_->getFixedFrame().toStdString()<<std::endl;

}




/*
//Image :
    grid_ = manager_->createDisplay( "rviz/Image", "Image View", true );
    ROS_ASSERT( grid_ != NULL );
    grid_->subProp( "Image Topic" )->setValue( "/camera/rgb/image_raw" );
    grid_->subProp( "Transport Hint" )->setValue( "theora" );


//Depth Cloud :
    grid_ = manager_->createDisplay( "rviz/DepthCloud", "Image View", true );
    ROS_ASSERT( grid_ != NULL );

    grid_->subProp( "Depth Map Topic" )->setValue( "/camera/depth/image_raw" );
    grid_->subProp( "Depth Map Transport Hint" )->setValue( "raw" );
    grid_->subProp( "Color Image Topic" )->setValue( "/camera/rgb/image_raw" );
    grid_->subProp( "Color Transport Hint" )->setValue( "raw" );
    grid_->subProp("Queue Size")->setValue(5);
    grid_->subProp("Style")->setValue("Flat Squares");

//    mainDisplay_ = manager_->createDisplay( "rviz/PointCloud2", "3D Pointcloud view", true );
//    ROS_ASSERT( mainDisplay_ != NULL );

//    mainDisplay_->subProp( "Topic" )->setValue( pointCloudTopic_ );
//    mainDisplay_->subProp( "Selectable" )->setValue( "true" );
//    mainDisplay_->subProp( "Style" )->setValue( "Boxes" );
//    mainDisplay_->subProp("Alpha")->setValue(0.5);

  manager_->createDisplay( "rviz/Grid", "Grid", true );

//MarkerArray :
rviz::Display* octomapDisplay_ = manager_->createDisplay( "rviz/MarkerArray", "Octomap view", true );
ROS_ASSERT( octomapDisplay_ != NULL );

octomapDisplay_->subProp( "Marker Topic" )->setValue( "/occupied_cells_vis_array" );


*/
