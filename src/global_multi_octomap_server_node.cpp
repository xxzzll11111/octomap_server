/**
* global_multi_octomap_server: A Tool to serve 3D OctoMaps in ROS (binary and as visualization)
* (inspired by the ROS map_saver)
* @author A. Hornung, University of Freiburg, Copyright (C) 2009 - 2012.
* @see http://octomap.sourceforge.net/
* License: BSD
*/

/*
 * Copyright (c) 2009-2012, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <ros/ros.h>
#include <octomap_server/MultiOctomapServer.h>
#include <octomap_server/TransformStampedArraywithMap.h>
#include <octomap_server/GetSubmaps.h>

#define USAGE "\nUSAGE: global_multi_octomap_server <map.[bt|ot]>\n" \
        "  map.bt: inital octomap 3D map file to read\n"

#ifdef COLOR_OCTOMAP_SERVER
  typedef octomap::ColorOcTree OcTreeT;
#else
  typedef octomap::OcTree OcTreeT;
#endif

using namespace octomap_server;

MultiOctomapServer* server_ptr;

void add_submap_to_global(const geometry_msgs::TransformStamped &tf_msg, const octomap_msgs::OctomapWithPose &submap_msg){
  tf::StampedTransform kfToWorldTf;
  tf::transformStampedMsgToTF(tf_msg, kfToWorldTf);

  Eigen::Matrix4f kfToWorld_;
  pcl_ros::transformAsMatrix(kfToWorldTf, kfToWorld_);
  Eigen::Affine3f kfToWorld(kfToWorld_);

  OcTreeT* submap = dynamic_cast<OcTreeT*>(octomap_msgs::msgToMap(submap_msg.octomap));
  for (OcTreeT::iterator it = submap->begin(submap->getTreeDepth()), end = submap->end(); it != end; ++it)
  {
      double x = it.getX();
      double y = it.getY();
      double z = it.getZ();
      pcl::PointXYZ p = pcl::transformPoint(pcl::PointXYZ(x, y, z), kfToWorld);
      float log_odds_value = it->getLogOdds();
      server_ptr->m_octree->updateNode(p.x, p.y, p.z , log_odds_value, false);
  }
}

void merge_Global_Map_Callback(const octomap_server::TransformStampedArraywithMap::ConstPtr &msg){
  std::cout << "start merge submap" << std::endl;
  server_ptr->m_octree->clear();
  server_ptr->publishAll(ros::Time::now());
  std::cout << "clear octree" << std::endl;
  std::vector<geometry_msgs::TransformStamped>::const_iterator tf;
  std::vector<octomap_msgs::OctomapWithPose>::const_iterator submap;
  for(submap=msg->submaps.begin(); submap!=msg->submaps.end(); submap++){
    for(tf=msg->transformArray.begin(); tf!=msg->transformArray.end(); tf++){
      if(tf->header.frame_id==submap->header.frame_id){
        add_submap_to_global(*tf, *submap);
        break;
      }
    }
  }
  server_ptr->publishAll(ros::Time::now());
}

int main(int argc, char** argv){
  ros::init(argc, argv, "octomap_server");
  const ros::NodeHandle nh;
  const ros::NodeHandle private_nh("~");
  std::string mapFilename(""), mapFilenameParam("");

  if (argc > 2 || (argc == 2 && std::string(argv[1]) == "-h")){
    ROS_ERROR("%s", USAGE);
    exit(-1);
  }

  MultiOctomapServer server(private_nh, nh);
  server_ptr = &server;
  ros::spinOnce();

  if (argc == 2){
    mapFilename = std::string(argv[1]);
  }

  if (private_nh.getParam("map_file", mapFilenameParam)) {
    if (mapFilename != "") {
      ROS_WARN("map_file is specified by the argument '%s' and rosparam '%s'. now loads '%s'", mapFilename.c_str(), mapFilenameParam.c_str(), mapFilename.c_str());
    } else {
      mapFilename = mapFilenameParam;
    }
  }

  if (mapFilename != "") {
    if (!server.openFile(mapFilename)){
      ROS_ERROR("Could not open file %s", mapFilename.c_str());
      exit(1);
    }
  }

  ros::Subscriber sub = server_ptr->m_nh.subscribe<octomap_server::TransformStampedArraywithMap>("TranswithSubMap", 1, merge_Global_Map_Callback);  //设置回调函数

  try{
    ros::spin();
  }catch(std::runtime_error& e){
    ROS_ERROR("global_multi_octomap_server exception: %s", e.what());
    return -1;
  }

  return 0;
}
