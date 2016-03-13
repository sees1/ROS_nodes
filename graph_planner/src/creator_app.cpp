/* 
 * File:  creator_app.cpp
 * Author: Vladislav Tananaev
 *
 * Created on February 22, 2016, 9:40 AM
 */

#include <cstdlib>
#include "graph_planner/creator_wrapper.h"

using namespace std;


int main(int argc, char** argv) {
    ros::init(argc, argv, "creator");


    CreatorWrapper creator;

    creator.init();

    ros::Rate r(10);
    unsigned int count = 0;
    while (ros::ok()) {
        ros::spinOnce();
        creator.spinOnes();
        if (count++ > 5) {
            count = 0;
           
           creator.pubRvizGraph();
        
        }

        r.sleep();
    }

}
