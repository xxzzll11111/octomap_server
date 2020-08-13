/**
* sub_octomap_server: A Tool to serve 3D OctoMaps in ROS (binary and as visualization)
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
#include <octomap_server/OctomapServer.h>
#include <octomap_server/GetSubmaps.h>
#include <octomap_msgs/OctomapWithPose.h>
#include <geometry_msgs/PoseStamped.h>

#define USAGE "\nUSAGE: sub_octomap_server <map.[bt|ot]>\n" \
        "  map.bt: inital octomap 3D map file to read\n"

using namespace octomap_server;

std::vector<octomap_msgs::OctomapWithPose> submaps;
OctomapServer* server_ptr;

void keyframe_Callback(const geometry_msgs::PoseStamped::ConstPtr &msg){
  if(submaps.size() == 0){
    octomap_msgs::OctomapWithPose submap;
    submaps.push_back(submap);
  }
  else if(submaps.back().header.frame_id != msg->header.frame_id){
    if(octomap_msgs::binaryMapToMsg(*(server_ptr->m_octree), submaps.back().octomap)){
      submaps.back().octomap.header.frame_id = server_ptr->m_worldFrameId;
      submaps.back().octomap.header.stamp = ros::Time::now();
      octomap_msgs::OctomapWithPose submap;
      submaps.push_back(submap);
      server_ptr->m_octree->clear();
    }
  }
  submaps.back().header = msg->header;
  submaps.back().origin = msg->pose;
}

bool get_submaps_Srv(octomap_server::GetSubmaps::Request &req, octomap_server::GetSubmaps::Response &res){
  if(!octomap_msgs::binaryMapToMsg(*(server_ptr->m_octree), submaps.back().octomap))
    return false;
  res.submaps = submaps;
  return true;
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

  OctomapServer server(private_nh, nh);
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

  ros::Subscriber sub = server_ptr->m_nh.subscribe<geometry_msgs::PoseStamped>("keyframe_pose", 1, keyframe_Callback);  //设置回调函数
  ros::ServiceServer srv = server_ptr->m_nh.advertiseService("get_submaps", get_submaps_Srv);

  try{
    ros::spin();
  }catch(std::runtime_error& e){
    ROS_ERROR("sub_octomap_server exception: %s", e.what());
    return -1;
  }

  return 0;
}
