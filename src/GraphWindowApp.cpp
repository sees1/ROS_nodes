
#include "GraphWindowApp.h"

GraphWindowApp::GraphWindowApp(QWidget *parent) : nh_(){
  
  // this sets up GUI
  setupUi(this); 

    //Connect point buttons in GUI with functions
    connect(newPointButton, SIGNAL(clicked()), this, SLOT(addNewPoint()));
    connect(deleteSelectedPointButton, SIGNAL(clicked()), this, SLOT(deleteSelectedPoint()));

    //Connect Edge buttons in GUI with functions
    connect(edgeBox, SIGNAL(currentIndexChanged(int)), this, SLOT(updateEdge(int)));
    connect(addUpdateButton, SIGNAL(clicked()), this, SLOT(addUpdate()));
    connect(deleteEdgeButton, SIGNAL(clicked()), this, SLOT(deleteEdge()));
    connect(saveButton, SIGNAL(clicked()), this, SLOT(saveGraph()));

    // Initialize default settings in the boxes
    edgeBox->addItem(QString::fromUtf8("add new edge"));
    fromBox->QComboBox::setCurrentIndex(fromBox->findText(""));
    toBox->QComboBox::setCurrentIndex(toBox->findText(""));
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


void GraphWindowApp::deleteSelectedPoint(){

 int index = pointBox->currentIndex();
    graph_planner::PointCmd msg;
    msg.cmd = PointCMD::del_point;
    msg.pointInfo.key_id = list_.pointList[index].key_id;
    cmd_point_pub_.publish(msg);

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

     //edge box
    indx_from_.clear();
    indx_to_.clear();
    for (unsigned int i = edgeBox->count(); i < list->edgeList.size() + 1; i++) {
        std::string from_name;
        std::string to_name;
        std::string name;

        for (unsigned int j = 0; j < list->pointList.size(); j++) {
            if (list->pointList[j].key_id == list->edgeList[i - 1].from_point) {
                from_name = list->pointList[j].name;

            }
            if (list->pointList[j].key_id == list->edgeList[i - 1].to_point) {
                to_name = list->pointList[j].name;

            }
        }
        name = "from " + from_name + " to " + to_name;
        edgeBox->addItem(QString::fromUtf8(name.c_str()));
        std::cerr << "Edge works: " << list->edgeList.size() << std::endl;
    }

    for (unsigned int i = edgeBox->count(); i > list->edgeList.size() + 1; i--) {
        edgeBox->removeItem(edgeBox->count() - 1);
    }

    for (unsigned int i = 1; i < edgeBox->count(); i++) {
        std::string from_name;
        std::string to_name;
        std::string name;
        for (unsigned int j = 0; j < list->pointList.size(); j++) {
            if (list->pointList[j].key_id == list->edgeList[i - 1].from_point) {
                from_name = list->pointList[j].name;
                indx_from_.push_back(j);
            }
            if (list->pointList[j].key_id == list->edgeList[i - 1].to_point) {
                to_name = list->pointList[j].name;
                indx_to_.push_back(j);
            }
        }
        name = "from " + from_name + " to " + to_name;
        edgeBox->setItemText(i, QString::fromUtf8(name.c_str()));
    }

    //from box
    for (unsigned int i = fromBox->count(); i < list->pointList.size(); i++) {
        std::string name = list->pointList[i].name;
        fromBox->addItem(QString::fromUtf8(name.c_str()));
    }

    for (unsigned int i = fromBox->count(); i > list->pointList.size(); i--) {
        fromBox->removeItem(fromBox->count() - 1);
    }


    for (unsigned int i = 0; i < fromBox->count(); i++) {
        std::string name = list->pointList[i].name;
        fromBox->setItemText(i, QString::fromUtf8(name.c_str()));
    }

    //to box
    for (unsigned int i = toBox->count(); i < list->pointList.size(); i++) {
        std::string name = list->pointList[i].name;
        toBox->addItem(QString::fromUtf8(name.c_str()));
    }

    for (unsigned int i = toBox->count(); i > list->pointList.size(); i--) {
        toBox->removeItem(toBox->count() - 1);
    }


    for (unsigned int i = 0; i < toBox->count(); i++) {
        std::string name = list->pointList[i].name;
        toBox->setItemText(i, QString::fromUtf8(name.c_str()));

    }

    updateEdge(edgeBox->currentIndex());
 


   //init box
    for (unsigned int i = initBox->count(); i < list->pointList.size(); i++) {
        std::string name = list->pointList[i].name;
        initBox->addItem(QString::fromUtf8(name.c_str()));
    }

    for (unsigned int i = initBox->count(); i > list->pointList.size(); i--) {
        initBox->removeItem(initBox->count() - 1);
    }


    for (unsigned int i = 0; i < initBox->count(); i++) {
        std::string name = list->pointList[i].name;
        initBox->setItemText(i, QString::fromUtf8(name.c_str()));

    }


       //goal box
    for (unsigned int i = goalBox->count(); i < list->pointList.size(); i++) {
        std::string name = list->pointList[i].name;
        goalBox->addItem(QString::fromUtf8(name.c_str()));
    }

    for (unsigned int i = goalBox->count(); i > list->pointList.size(); i--) {
        goalBox->removeItem(goalBox->count() - 1);
    }


    for (unsigned int i = 0; i < goalBox->count(); i++) {
        std::string name = list->pointList[i].name;
        goalBox->setItemText(i, QString::fromUtf8(name.c_str()));

    }

}

void GraphWindowApp::updateEdge(int index) {
    if (index == 0) {
        fromBox->QComboBox::setCurrentIndex(fromBox->findText(""));
        toBox->QComboBox::setCurrentIndex(toBox->findText(""));
        weightBox->setValue(1.0);
    } else if (index >= 1) {
        fromBox->setCurrentIndex(fromBox->findText(QString::fromUtf8(list_.pointList[indx_from_[index - 1]].name.c_str())));
        toBox->setCurrentIndex(toBox->findText(QString::fromUtf8(list_.pointList[indx_to_[index - 1]].name.c_str())));
        weightBox->setValue(list_.edgeList[index-1].weight);
    }

}

void GraphWindowApp::addUpdate(){

     int index = edgeBox->currentIndex();
    int index_from = fromBox->currentIndex();
    int index_to = toBox->currentIndex();
    double weight = weightBox->value();
    // add
    if (index == 0 && index_from != index_to) {
        graph_planner::EdgeCmd msg;
        msg.cmd = EdgeCMD::add_edge;
        msg.edgeInfo.from_point = list_.pointList[index_from].key_id;
        msg.edgeInfo.to_point = list_.pointList[index_to].key_id;
        msg.edgeInfo.weight = weight;
        cmd_edge_pub_.publish(msg);
    }//update
    else if (index >= 1 && index_from != index_to) {
        graph_planner::EdgeCmd msg;
        msg.cmd = EdgeCMD::update_edge;
        msg.edgeInfo.key_id = list_.edgeList[index - 1].key_id;
        msg.edgeInfo.from_point = list_.pointList[index_from].key_id;
        msg.edgeInfo.to_point = list_.pointList[index_to].key_id;
        msg.edgeInfo.weight = weight;
        cmd_edge_pub_.publish(msg);
    }
}

void GraphWindowApp::deleteEdge(){

 int index = edgeBox->currentIndex();
    if (index > 0) {
        graph_planner::EdgeCmd msg;
        msg.cmd =EdgeCMD::del_edge;
        msg.edgeInfo.key_id = list_.edgeList[index - 1].key_id;
        cmd_edge_pub_.publish(msg);
    }
}

void GraphWindowApp::saveGraph(){
    graph_planner::EdgeCmd msg;
    msg.cmd = EdgeCMD::save_graph;
    cmd_edge_pub_.publish(msg);
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
