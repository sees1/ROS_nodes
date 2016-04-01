

#ifndef INTERCOM2_H
#define	INTERCOM2_H

#include "intercom2/relay_topic.h"

namespace ros {
    namespace master {
        void init(const M_string& remappings);
    }
}



//The class with functions which read the parameters from launch file and subscribe topics
class multimaster {
public:
    multimaster();
    ~multimaster();
     bool getParam();
     std::string foreign_master_uri();
     void init(ros::M_string remappings);
    bool getHostTopicsList();
    bool getForeignTopicsList();
    void host2foreign(ros::M_string remappings);
    void foreign2host(ros::M_string remappings);

    std::vector<std::string> hostTopicsList;
    std::vector<std::string> foreignTopicsList;

     std::string namesp;
    std::string foreign_master, host_master;
     std::string foreign_ip;
     int foreign_port, msgsFrequency_Hz;
     ros::NodeHandle nh;
private:

    std::string  config_name_;
    std::string  folder_path_;

};

#endif	/* INTERCOM2_H */
