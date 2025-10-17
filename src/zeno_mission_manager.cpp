/*#########################################################################################################

----------------------------------------------------------------------------------------------------------
Made by: Matteo Bresciani          Email: matteo.bresciani@phd.unipi.it            Date:  13/03/2023
#########################################################################################################*/
#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <unistd.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "std_msgs/Empty.h"
#include "std_msgs/String.h"

#include "marta_msgs/MissionManager.h"
#include "marta_msgs/NavStatus.h"
#include "zeno_mission_manager/Waypoint.h"
#include "zeno_mission_manager/WaypointList.h"

using namespace std;
using namespace Eigen;

// the mission manager is handled with 5 states
// IDLE --> no mission loaded
// READY --> mission loaded but not executed yet (mission can still be overwritten)
// PAUSED --> mission loaded and started, but in pause
// RUNNING --> mission loaded, started and currently running
// COMPLETED --> mission completed
enum STATUS { IDLE, READY, PAUSED, RUNNING, COMPLETED};
std::map<STATUS, std::string> status_table = {
                                              {STATUS::IDLE, "IDLE"},      
                                              {STATUS::READY, "READY"},      
                                              {STATUS::PAUSED, "PAUSED"},      
                                              {STATUS::RUNNING, "RUNNING"},     
                                              {STATUS::COMPLETED, "COMPLETED"}     
                                              };
const int MIN_DEPTH_THRESHOLD     = 0;    //[m]
const int MAX_DEPTH_THRESHOLD     = 10;  //[m]
const int MIN_ALTITUDE_THRESHOLD  = 2;    //[m]
const int MAX_ALTITUDE_THRESHOLD  = 100;  //[m]
const int MAX_WAYPOINT_DISTANCE   = 1000; //[m]
const double MAX_SPEED            = 1.02; //[m/s]
const double MIN_SPEED            = 0.102;//[m/s]

string          control_mode;
int             seq_counter = 0;
STATUS          current_status = STATUS::IDLE;
Vector2d        vehicle_position;
ros::Publisher  zeno_xml_pub, mission_status_pub;

//---------------------------------------------------------------------
//---------------------------------------------------------------------
Vector2d ll2ne(const Vector2d& ll0, const Vector2d& ll)
{      
  double lat0 = ll0(0);
  double lon0 = ll0(1);
  double lat  = ll(0);
  double lon  = ll(1);

  Vector2d ne = Vector2d::Zero();

  lat   = lat * M_PI/180;
  lon   = lon * M_PI/180;
  lat0  = lat0 * M_PI/180;
  lon0  = lon0 * M_PI/180;

  double dlat = lat - lat0;
  double dlon = lon - lon0;

  double a = 6378137.0;
  double f = 1 / 298.257223563;
  double Rn = a / sqrt(1 - (2 * f - f * f) * sin(lat0) * sin(lat0));
  double Rm = Rn * (1 - (2 * f - f * f)) / (1 - (2 * f - f * f) * sin(lat0) * sin(lat0));

  // This are the ned equivalents, for convenience I save them in the same variable
  ne(0) = dlat / atan2(1, Rm);
  ne(1) = dlon / atan2(1, Rn * cos(lat0));

  return ne;      
}
//---------------------------------------------------------------------
//---------------------------------------------------------------------
double ll2distance( const Vector2d& ll0, const Vector2d& ll) 
{
  Vector2d tmp_ned = ll2ne(ll0, ll);
  return sqrt( pow(tmp_ned(1),2) + pow(tmp_ned(0),2) );
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
void publishMissionStatus()
{
  ROS_INFO_STREAM("[ZENO MISSION MANAGER - publishMissionStatus] Mission is " << status_table[current_status] <<"!");
  std_msgs::String msg_out;
  msg_out.data = status_table[current_status];
  mission_status_pub.publish(msg_out);
}
//---------------------------------------------------------------------
//---------------------------------------------------------------------
void uploadMissionCallback(const zeno_mission_manager::WaypointList::ConstPtr& msg)
{
  //~~~~~~~~~~~~~~~~~~~~~~~~~~
  // EDIT COMMAND
  // <xml>
  // <mission_edit seq=\"1\"/>
  // <event_request trigger=\"control_mode\" params=\"set={depth}\"/>
  // <move_geo mission=\"default\" lat=\"42.8244261\" lon=\"10.2686693\" depth=\"0\" hor_speed=\"0.257222\" point_of_view=\"front\"/>
  // <event_request trigger=\"control_mode\" params=\"set={depth}\"/>
  // <move_geo mission=\"default\" lat=\"42.8240389\" lon=\"10.2682737\" depth=\"0\" hor_speed=\"0.257222\" point_of_view=\"front\"/>
  // <mission_resume mission=\"default\" seq=\"3\"/>
  // <mission_resume mission=\"pause\" seq=\"2\"/>
  // </xml>
  //~~~~~~~~~~~~~~~~~~~~~~~~~~
  // check on status
  if (current_status == STATUS::RUNNING || current_status == STATUS::PAUSED)
  {
    ROS_WARN_STREAM_COND(current_status == STATUS::RUNNING, "[ZENO MISSION MANAGER - uploadMissionCallback] The current status of Zeno is: " << status_table[current_status] <<"; cannot load new mission while one is running. Delete running mission prior uploading a new one.");
    ROS_WARN_STREAM_COND(current_status == STATUS::PAUSED, "[ZENO MISSION MANAGER - uploadMissionCallback] The current status of Zeno is: " << status_table[current_status] <<"; cannot load new mission while one is paused. Delete paused mission prior uploading a new one.");
    return;
  }

  // check on data
  zeno_mission_manager::WaypointList mission = *msg;
  for(int i=0; i< mission.waypoint_list.size(); i++)
  {
    // check on depth or altitude mode and values
    if (mission.waypoint_list[i].control_mode == "depth")
    {
      mission.waypoint_list[i].position.depth = mission.waypoint_list[i].position.depth < MIN_DEPTH_THRESHOLD ? MIN_DEPTH_THRESHOLD : mission.waypoint_list[i].position.depth;
      mission.waypoint_list[i].position.depth = mission.waypoint_list[i].position.depth > MAX_DEPTH_THRESHOLD ? MAX_DEPTH_THRESHOLD : mission.waypoint_list[i].position.depth;
    }
    else if (mission.waypoint_list[i].control_mode == "altitude")
    {
      mission.waypoint_list[i].altitude = mission.waypoint_list[i].altitude < MIN_ALTITUDE_THRESHOLD ? MIN_ALTITUDE_THRESHOLD : mission.waypoint_list[i].altitude;
      mission.waypoint_list[i].altitude = mission.waypoint_list[i].altitude > MAX_ALTITUDE_THRESHOLD ? MAX_ALTITUDE_THRESHOLD : mission.waypoint_list[i].altitude;
      mission.waypoint_list[i].position.depth = mission.waypoint_list[i].altitude;
    }
    else
    {
      ROS_ERROR_STREAM("[ZENO MISSION MANAGER - uploadMissionCallback] Unkown control mode selected for the ["<< i <<"] waypoint ["<< mission.waypoint_list[i].control_mode <<"]. Cannot generate new mission.");
      return;
    }

    // check on the distance between the waypoint and the current vehicle position
    Vector2d waypoint;
    waypoint << mission.waypoint_list[i].position.latitude, mission.waypoint_list[i].position.longitude;
    double distance_wp_vehicle = ll2distance(waypoint, vehicle_position);
    if (distance_wp_vehicle > MAX_WAYPOINT_DISTANCE )
    {
      ROS_ERROR_STREAM("[ZENO MISSION MANAGER - uploadMissionCallback] Waypoint ["<< i <<"] is too far (>1km) from the current vehicle position. Cannot generate the mission for safety reasons.");
      return;
    }

    // check on the speed
    mission.waypoint_list[i].speed = mission.waypoint_list[i].speed < MIN_SPEED ? MIN_SPEED : mission.waypoint_list[i].speed;
    mission.waypoint_list[i].speed = mission.waypoint_list[i].speed > MAX_SPEED ? MAX_SPEED : mission.waypoint_list[i].speed;      
  }


  // generate the command
  std_msgs::String msg_out;
  stringstream ss;
  ss << "<xml><mission_edit seq=\""<< seq_counter++ << "\"/>";  
  for(int i=0; i< mission.waypoint_list.size(); i++)
  {
    if (mission.waypoint_list[i].control_mode == "depth")
      ss << "<event_request trigger=\"control_mode\" params=\"set={"<< mission.waypoint_list[i].control_mode <<"}\"/>\n";
    else 
      ss << "<event_request trigger=\"control_mode\" params=\"set={"<< mission.waypoint_list[i].control_mode <<"}|setpoint={"<< mission.waypoint_list[i].altitude <<"}\"/>\n";

    ss << "<move_geo mission=\"default\" lat=\""<< mission.waypoint_list[i].position.latitude;
    ss <<"\" lon=\""<< mission.waypoint_list[i].position.longitude <<"\" depth=\""<< mission.waypoint_list[i].position.depth;
    ss <<"\" hor_speed=\""<< mission.waypoint_list[i].speed <<"\" point_of_view=\"front\"/>";
  }
  // ss << "<mission_resume mission=\"default\" seq=\""<< seq_counter++ << "\"/>";
  // ss << "<mission_resume mission=\"pause\" seq=\""<< seq_counter++ << "\"/>";
  ss << "</xml>";  

  msg_out.data = ss.str();
  zeno_xml_pub.publish(msg_out);
  current_status = STATUS::READY;
  publishMissionStatus();
  // ROS_INFO_STREAM("[ZENO MISSION MANAGER - statusMissionCallback] Mission is " << status_table[current_status] <<"!");
}
//---------------------------------------------------------------------
//---------------------------------------------------------------------
void startMissionCallback(const std_msgs::Empty::ConstPtr& msg)
{
  //~~~~~~~~~~~~~~~~~~~~~~~~~~
  // START COMMAND
  // <mission_resume mission=\"default\" seq=\"4\"/>
  //~~~~~~~~~~~~~~~~~~~~~~~~~~
  if (current_status == STATUS::IDLE || current_status == STATUS::RUNNING)
  {
    ROS_WARN_STREAM_COND(current_status == STATUS::IDLE, "[ZENO MISSION MANAGER - startMissionCallback] The current status of Zeno is: " << status_table[current_status] <<"; no loaded mission to run.");
    ROS_WARN_STREAM_COND(current_status == STATUS::RUNNING, "[ZENO MISSION MANAGER - startMissionCallback] The current status of Zeno is: " << status_table[current_status] <<"; mission already running.");
    return;
  }
  std_msgs::String msg_out;
  stringstream ss;
  ss << "<mission_resume mission=\"default\" seq=\""<< seq_counter++ << "\"/>";
  msg_out.data = ss.str();
  zeno_xml_pub.publish(msg_out);
}
//---------------------------------------------------------------------
//---------------------------------------------------------------------
void pauseMissionCallback(const std_msgs::Empty::ConstPtr& msg)
{
  //~~~~~~~~~~~~~~~~~~~~~~~~~~
  // PAUSE COMMAND
  // <pause seq=\"5\"/>
  //~~~~~~~~~~~~~~~~~~~~~~~~~~
  if (current_status != STATUS::RUNNING)
  {
    ROS_WARN_STREAM("[ZENO MISSION MANAGER - pauseMissionCallback] The current status of Zeno is: " << status_table[current_status] <<"; no running mission to pause.");
    return;
  }
  std_msgs::String msg_out;
  stringstream ss;
  ss << "<pause seq=\""<< seq_counter++ << "\"/>";
  msg_out.data = ss.str();
  zeno_xml_pub.publish(msg_out);
}
//---------------------------------------------------------------------
//---------------------------------------------------------------------
void deleteMissionCallback(const std_msgs::Empty::ConstPtr& msg)
{
  //~~~~~~~~~~~~~~~~~~~~~~~~~~
  // DELETE COMMAND
  // <mission_reset mission=\"default\"/>
  //~~~~~~~~~~~~~~~~~~~~~~~~~~
  if (current_status == STATUS::RUNNING || current_status == STATUS::IDLE)
  {
    ROS_WARN_STREAM("[ZENO MISSION MANAGER - deleteMissionCallback] The current status of Zeno is: " << status_table[current_status] <<"; cannot delete the mission.");
    return;
  }

  std_msgs::String msg_out;
  msg_out.data = "<mission_reset mission=\"default\"/>";
  zeno_xml_pub.publish(msg_out);
  current_status = STATUS::IDLE;
  publishMissionStatus();

}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
void statusMissionCallback(const marta_msgs::MissionManager::ConstPtr& msg)
{
  if(msg->mission_running == "pause" && current_status != STATUS::IDLE && current_status != STATUS::READY && current_status != STATUS::PAUSED)
  {
    current_status = STATUS::PAUSED;
    publishMissionStatus();
    // ROS_INFO_STREAM("[ZENO MISSION MANAGER - statusMissionCallback] Mission is " << status_table[current_status] <<"!");
  }
  else if(msg->mission_running == "default" && msg->is_completed && current_status != STATUS::COMPLETED)
  {
    current_status = STATUS::COMPLETED;
    publishMissionStatus();
    // ROS_INFO_STREAM("[ZENO MISSION MANAGER - statusMissionCallback] Mission is " << status_table[current_status] <<"!");
  }
  else if(msg->mission_running == "default" && !msg->is_completed && current_status != STATUS::RUNNING)
  {
    current_status = STATUS::RUNNING;
    publishMissionStatus();
    // ROS_INFO_STREAM("[ZENO MISSION MANAGER - statusMissionCallback] Mission is " << status_table[current_status] <<"!");
  }
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
void navstatusCallback(const marta_msgs::NavStatus::ConstPtr& msg)
{
  vehicle_position(0) = msg->position.latitude;
  vehicle_position(1) = msg->position.longitude;
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
//              MAIN
//---------------------------------------------------------------------
//---------------------------------------------------------------------
int main(int argc, char **argv)
{  
  ros::init(argc, argv, "zeno_mission_manager");
  ros::NodeHandle n;
  
  // CREATE ROS PUBLISHERs
  zeno_xml_pub        = n.advertise<std_msgs::String>("/refgen/xml/XMLCommand", 1);
  mission_status_pub  = n.advertise<std_msgs::String>("mm/mission_status", 1);

  // CREATE ROS SUBSCRIBERs
  ros::Subscriber upload_mission_sub    = n.subscribe<zeno_mission_manager::WaypointList>("mm/upload_mission",10, uploadMissionCallback);
  ros::Subscriber delete_mission_sub    = n.subscribe<std_msgs::Empty>("mm/delete_mission",10, deleteMissionCallback);
  ros::Subscriber pause_mission_sub     = n.subscribe<std_msgs::Empty>("mm/pause_mission",10, pauseMissionCallback);
  ros::Subscriber start_mission_sub     = n.subscribe<std_msgs::Empty>("mm/start_mission",10, startMissionCallback);

  ros::Subscriber status_mission_sub    = n.subscribe<marta_msgs::MissionManager>("/refgen/xml/Status",10, statusMissionCallback);
  ros::Subscriber navstatus_sub         = n.subscribe<marta_msgs::NavStatus>("/nav_status",10, navstatusCallback);

  sleep(1);
  publishMissionStatus();
  ros::spin();
}