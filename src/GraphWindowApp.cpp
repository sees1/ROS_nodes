
#include "GraphWindowApp.h"

GraphWindowApp::GraphWindowApp(QWidget *parent) : nh_(){
  
  // this sets up GUI
  setupUi(this); 

    //Connect point buttons in GUI with functions
    connect(newPointButton, SIGNAL(clicked()), this, SLOT(addNewPoint()));
    connect(updatePointButton, SIGNAL(clicked()), this, SLOT(updatePoint()));
    connect(deleteSelectedPointButton, SIGNAL(clicked()), this, SLOT(deleteSelectedPoint()));
    connect(pointBox, SIGNAL(currentIndexChanged(int)), this, SLOT(updatePointbox(int)));
}


GraphWindowApp::~GraphWindowApp(){  
}


void GraphWindowApp::init() {
    graph_sub_ = nh_.subscribe<graph_planner::GraphStructure> ("/graph", 1, &GraphWindowApp::graphCallback, this);
    cmd_point_pub_ = nh_.advertise < graph_planner::PointCmd> ("point_cmd", 0);
    cmd_edge_pub_ = nh_.advertise < graph_planner::EdgeCmd> ("edge_cmd", 0);     
}

void GraphWindowApp::addNewPoint(){

   graph_planner::PointCmd msg;
    msg.cmd = PointCMD::new_point;
    msg.pointInfo.x = 0;
    msg.pointInfo.y = 0;
    msg.pointInfo.name = "";
    cmd_point_pub_.publish(msg);
}

void GraphWindowApp::updatePoint(){

 int index = pointBox->currentIndex();

    if (index >= 0 && index < list_.pointList.size()) {
        graph_planner::PointCmd msg;
        msg.cmd = PointCMD::update_point;
        msg.pointInfo.key_id = list_.pointList[index].key_id;
        msg.pointInfo.x = xCoord->value();
        msg.pointInfo.y = yCoord->value();
        std::string name = nameField->text().toStdString();
        msg.pointInfo.name = name;;
        cmd_point_pub_.publish(msg);
    }

}

void GraphWindowApp::deleteSelectedPoint(){

 int index = pointBox->currentIndex();
    graph_planner::PointCmd msg;
    msg.cmd = PointCMD::del_point;
    msg.pointInfo.key_id = list_.pointList[index].key_id;
    cmd_point_pub_.publish(msg);

}

void GraphWindowApp::updatePointbox(int index){

        if (index >= 0) {
        xCoord->setValue(list_.pointList[index].x);
        yCoord->setValue(list_.pointList[index].y);
        nameField->setText(QString::fromUtf8(list_.pointList[index].name.c_str()));
    }
}

void GraphWindowApp::graphCallback(const  graph_planner::GraphStructure::ConstPtr& list){
list_ = *list;

    //point box
    for (unsigned int i = pointBox->count(); i < list->pointList.size(); i++) {
        std::string name = list->pointList[i].name;
        pointBox->addItem(QString::fromUtf8(name.c_str()));
    }

    for (unsigned int i = pointBox->count(); i > list->pointList.size(); i--) {
        pointBox->removeItem(pointBox->count() - 1);
    }


    for (unsigned int i = 0; i < pointBox->count(); i++) {
        std::string name = list->pointList[i].name;
        pointBox->setItemText(i, QString::fromUtf8(name.c_str()));
    }

    updatePointbox(pointBox->currentIndex());


}

int main(int argc, char** argv) {
  ros::init(argc, argv, "graph_test_gui", ros::init_options::AnonymousName);

  QApplication qapp(argc, argv);
  ros::NodeHandle nh_local("~");

  GraphWindowApp graph_window;
  graph_window.init();
  graph_window.show();
  
  while(ros::ok() && graph_window.isVisible()) {
    ros::spinOnce();
    qapp.processEvents();
  }
  return 0;
}
