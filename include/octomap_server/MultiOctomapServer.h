#ifndef OCTOMAP_SERVER_MULTIOCTOMAPSERVER_H
#define OCTOMAP_SERVER_MULTIOCTOMAPSERVER_H

#include <octomap_server/OctomapServer.h>

namespace octomap_server {

class MultiOctomapServer : public OctomapServer
{
    public:
    MultiOctomapServer(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_);
    ~MultiOctomapServer(){};

    protected:
    ros::Subscriber m_OctoMapSub;
    void mergeMapCallback(const octomap_msgs::Octomap::ConstPtr& cloud);
};

}

#endif