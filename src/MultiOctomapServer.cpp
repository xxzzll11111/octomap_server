#include <octomap_server/MultiOctomapServer.h>

using namespace octomap;

namespace octomap_server{
    
MultiOctomapServer::MultiOctomapServer(const ros::NodeHandle private_nh_ = ros::NodeHandle("~"), const ros::NodeHandle &nh_ = ros::NodeHandle())
: OctomapServer(private_nh_, nh_)
{
  m_OctoMapSub = m_nh.subscribe<octomap_msgs::Octomap>("octomap_in", 5, &MultiOctomapServer::mergeMapCallback, this);
  std::cout << "start listen map" << std::endl;
  ros::spin();
}

void MultiOctomapServer::mergeMapCallback(const octomap_msgs::Octomap::ConstPtr& map){
  std::cout << "start merge" << std::endl;
  tf::StampedTransform sensorToWorldTf;
  m_tfListener.waitForTransform(m_worldFrameId, map->header.frame_id, ros::Time(0), ros::Duration(4.0));
  try {
    m_tfListener.lookupTransform(m_worldFrameId, map->header.frame_id, ros::Time(0), sensorToWorldTf);
  } catch(tf::TransformException& ex){
    ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
    std::cout << "no tf " << m_worldFrameId << " to " << map->header.frame_id << " at stamp:" << ros::Time(0) << std::endl;
    return;
  }
  std::cout << "got tf" << m_worldFrameId << " to " << map->header.frame_id << std::endl;
  Eigen::Matrix4f sensorToWorld_;
  pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld_);
  Eigen::Affine3f sensorToWorld(sensorToWorld_);

  OcTreeT* other_robot_map = dynamic_cast<OcTreeT*>(octomap_msgs::msgToMap(*map));
  std::cout << "got map" << std::endl;

  for (OcTreeT::iterator it = other_robot_map->begin(m_maxTreeDepth), end = other_robot_map->end(); it != end; ++it)
  {
      double x = it.getX();
      double y = it.getY();
      double z = it.getZ();
      pcl::PointXYZ p = pcl::transformPoint(pcl::PointXYZ(x, y, z), sensorToWorld);
      float log_odds_value = it->getLogOdds();
      m_octree->updateNode(p.x, p.y, p.z , log_odds_value, false);
  }
  std::cout << "finish merge" << std::endl;
}

}