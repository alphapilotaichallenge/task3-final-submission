/*******************************************************************************
* main.cpp
* This program is proprietary to Team Titans of AlphaPilot Challenge
* Authors: Chris Mayer, Bhavyansh Mishra
* Captain: Ashok Yannam
*******************************************************************************/

#include <flightgoggles/IRMarkerArray.h>
#include <mav_msgs/RateThrust.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <vector>

void pilotUpdate(void);

typedef struct {
  double x, y;
} VECTOR2;

bool collision_flag = false;
std::vector<flightgoggles::IRMarkerArray> ir_markersVec;
mav_msgs::RateThrust rate_thrust;
sensor_msgs::Imu imu;
VECTOR2 screen;

// Path planner kept making an unexpected turn - wasn't a bug!
// Based on gate probabilities, it was makimizing ER..
//#define NUM_GATES 11
//int gate_sequence[NUM_GATES] =  {10, 21, 2, 13, 9, 14, 1, 22, 15, 23, 6};
#define NUM_GATES 6
int gate_sequence[NUM_GATES] =  {10, 21, 2, 13, 23, 6};

double gate_nominal_position[NUM_GATES][4][3];
char buffer[256];

void CollisionCallback(const std_msgs::Empty::ConstPtr &msg) {
  ROS_INFO("CollisionCallback----------------------------------------");
  collision_flag = true;
}

void IrBeaconsCallback(const flightgoggles::IRMarkerArrayConstPtr &msg) {  
  if(ir_markersVec.size() >= 10){
    ir_markersVec.erase(ir_markersVec.begin(), ir_markersVec.begin() + 1);
  }
  ir_markersVec.push_back(*msg);
}

void ImuCallback(const sensor_msgs::Imu &msg) {
  imu = msg;
}

void CameraCallback(const sensor_msgs::CameraInfo &msg) {
  screen.x = msg.width;
  screen.y = msg.height;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");
  ros::NodeHandle rnh;
  ros::Subscriber collision_sub = rnh.subscribe("/uav/collision", 1,
      CollisionCallback);
  ros::Subscriber beacons_sub = rnh.subscribe("/uav/camera/left/ir_beacons", 1,
      IrBeaconsCallback);
  ros::Subscriber imu_sub = rnh.subscribe("/uav/sensors/imu", 1, ImuCallback);
  ros::Subscriber camera_sub = rnh.subscribe("/uav/camera/left/camera_info", 1,
      CameraCallback);
  ros::Publisher pub_vel = rnh.advertise<mav_msgs::RateThrust>(
      "output/rateThrust", 1);
  ros::Rate loop_rate(60);
  // Give FG / Unity time to load
  usleep(6000000);
  // TODO: This is only need for the RL & GA training, not the pilot
  XmlRpc::XmlRpcValue my_list;
  for (int i = 0; i < NUM_GATES; i++) {
    sprintf(buffer, "/uav/Gate%d/nominal_location", gate_sequence[i]);
    rnh.getParam (buffer, my_list);
    for (int j = 0; j < my_list.size(); j++) {
      for (int k = 0; k < my_list[j].size(); k++) {
        gate_nominal_position[i][j][k] = static_cast<double>(my_list[j][k]);
      }
    }
  }
  ROS_INFO("------------%p", ir_markersVec);
  while (ros::ok()) {
    pilotUpdate();
    rate_thrust.header.frame_id = "uav/imu";
    rate_thrust.header.stamp    = ros::Time::now();
    pub_vel.publish(rate_thrust);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
