/*******************************************************************************
* pidPilot.cpp
* This program is proprietary to Team Titans of AlphaPilot Challenge
* Authors: Chris Mayer, Bhavyansh Mishra
* Captain: Ashok Yannam
*******************************************************************************/

#include <unordered_map>
#include <flightgoggles/IRMarkerArray.h>
#include <mav_msgs/RateThrust.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sys/time.h>
#include <math.h>

// Dependencies for Visual Pose Estimator
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"

//#define NUM_GATES 11 - see comment in main.cpp
#define NUM_GATES 6

typedef struct {
  double x, y;
} VECTOR2;

typedef struct {
  int target_index;
  VECTOR2 midpoint, lgm, corner[4], lgc[4];
  cv::Mat transform;
  double length, distance;
  bool found;
  bool passed;
} GATE_INFO;

typedef struct {
	double r, p, y, t;
} RATE_THRUST;

extern bool collision_flag;
extern mav_msgs::RateThrust rate_thrust;
extern int gate_sequence[NUM_GATES];
extern double gate_nominal_position[NUM_GATES][4][3];
extern char buffer[256];
extern VECTOR2 screen;
extern std::vector<flightgoggles::IRMarkerArray> ir_markersVec;
extern sensor_msgs::Imu imu;

std::string timestamp = std::to_string(time(NULL));

int num_runs = 0;
int num_passed_gates = 0;
std::unordered_map<std::string, int> stats = {
	{"Gate10", 0},
	{"Gate21", 0},
	{"Gate2", 0},
	{"Gate13", 0},
	{"Gate9", 0},
	{"Gate14", 0},
	{"Gate1", 0},
	{"Gate22", 0},
	{"Gate15", 0},
	{"Gate23", 0},
	{"Gate6", 0},
}; 

std::vector<std::string> gate_names = {
	"Gate10", "Gate21", "Gate2", "Gate13", "Gate23", "Gate6"
};

/*
DO NOT MODIFY THESE TABLES - This data came from RL/GA trainer model
*/
double fudgefactor[NUM_GATES] = { -100, 0, 50, -20, 0, -100 };

RATE_THRUST gate_looking_default[NUM_GATES] = {
  0, 0, 0, 0, // Gate10
  0, 0, 0, 0, // Gate21
  -0.80, 0.0, 1.5, 10,  // Gate2
  -0.5, 0, 1, 10, // Gate13
  1.3, 0, -2.6, 12, // Gate23 from 9
  1.1, 0, -2.2, 15  // Gate6
};

double lrnY[NUM_GATES] = {5,5,5,5,7.5,5};

enum {
  GATE_LOOKING,
  GATE_FOUND,
  GATE_PASSED,
  RACE_FINISHED
};

int gate_target = 0;
int gate_state = GATE_LOOKING;
double prev_length = -1;
int wait_timer = 0;

void pilotInit(void) {
}

double distance(VECTOR2 a, VECTOR2 b) {
  double dx = b.x - a.x;
  double dy = b.y - a.y;
  return sqrt((dx*dx) + (dy*dy));
}

// Velocity estimation code start 

// Drone position history vector
std::vector<std::array<double, 3>> dronePoseWithRespectToGateVector;
std::vector<double> droneDistWithRespectToGateVector;
double lastGoodVelocity = 0;
double lastGoodDistance = 0;

/* Save position of drone w.r.t  Gate in view*/
void saveGatePoseToBuffer(double x, double y, double z){
    std::array<double, 3> item;
    item[0]= x;
    item[1]= y;
    item[2]= z;
    
    //char pos_buffer [50];
    //sprintf(pos_buffer, " x %f y %f z %f", x, y, z);
    //std::cout << " " << pos_buffer <<" \n";
    
    if(dronePoseWithRespectToGateVector.size() >= 10){
        dronePoseWithRespectToGateVector.erase(dronePoseWithRespectToGateVector.begin(), dronePoseWithRespectToGateVector.begin() + 1);
    }
   
    if(droneDistWithRespectToGateVector.size() >= 10){
        droneDistWithRespectToGateVector.erase(droneDistWithRespectToGateVector.begin(), droneDistWithRespectToGateVector.begin() + 1);
    }  
  
    double currDist = lastGoodDistance;
    //if( std::isnan(x) ||  std::isnan(y) ||  std::isnan(z)){
    //   currDist = lastGoodDistance;
    //}else{
    //   currDist = sqrt( x*x + y*y + z*z ); 
    //}

    char pos_buffer [50];
    sprintf(pos_buffer, " Distance %f ", currDist);
    //std::cout << " " << pos_buffer <<" \n";

    // drone distance history vector
    droneDistWithRespectToGateVector.push_back(currDist);
    // Pose history vector
    dronePoseWithRespectToGateVector.push_back(item);
}


/* Estimate velocity based on position of drone with respect to Gate*/
double getDroneVelocity(){
    int size = droneDistWithRespectToGateVector.size();
    if(size <= 1) return 0;
    
    //std::array<double, 3> start = dronePoseWithRespectToGateVector[0];
    //std::array<double, 3> end = dronePoseWithRespectToGateVector[size-1];
   
    //double start_dist_to_gate = sqrt( start[0] * start[0] + start[1] * start[1] + start[2] * start[2] );
    //double end_dist_to_gate = sqrt( end[0] * end[0] + end[1] * end[1] + end[2] * end[2] );
    
    double start_dist_to_gate = droneDistWithRespectToGateVector[0];
    double end_dist_to_gate =  droneDistWithRespectToGateVector[size-1];
    
    double currentVelocity = (start_dist_to_gate - end_dist_to_gate)/ (size-1);
     
    if( std::isnan(currentVelocity)) return lastGoodVelocity;
    
    lastGoodVelocity = currentVelocity;
 
    return currentVelocity;
}
// Velocity estimation code ends


// Fill missing corners
void FillAndGetClockwiseCorners(std::vector<VECTOR2>& clockwise_corners, GATE_INFO* gate_info) {
  VECTOR2 top_left, top_right, bot_left, bot_right;
  VECTOR2 midpoint = gate_info->midpoint;
  bool tl_set = false, tr_set = false, bl_set = false, br_set = false;
  
  for (int i = 0; i < 4; i++) {
    VECTOR2 cur_corner = gate_info->corner[i];
    if (cur_corner.x <= midpoint.x) {
	// left
	if (cur_corner.y <= midpoint.y) {
		//top
		top_left = cur_corner;
		tl_set = true;
	} else {
		// bot
		bot_left = cur_corner;
		bl_set = true;
	}
    } else {
	// right
	if (cur_corner.y <= midpoint.y) {
		// top
		top_right = cur_corner;
		tr_set = true;
	} else {
		bot_right = cur_corner;
		br_set = true;
	}
    }
  }

  if (!tl_set) {
	//std::cout << "Fill: Missing top left!" << std::endl;
        float right_diff = top_right.x-bot_right.x; // shift left or right
	float bot_diff = bot_left.y-bot_right.y; // shift up or down

        VECTOR2 top_left = {bot_left.x+right_diff, top_right.y+bot_diff};
		
  }
  if (!tr_set) {
	//std::cout << "Fill: Missing top right!" << std::endl;
 	/*float bot_line_len = distance(bot_left, bot_right);
	float left_line_len = distance(top_left, bot_left);
	float bot_line_slope = get_slope(bot_left, bot_right);
	float left_line_slope = get_slope(top_left, bot_left);
	*/
	float left_diff = top_left.x-bot_left.x;
	float bot_diff = bot_right.y-bot_left.y; 

	VECTOR2 top_right = {bot_right.x+left_diff, top_left.y+bot_diff};

  }
  if (!bl_set) {
	//std::cout << "Fill: Missing bottom left!" << std::endl;
 	/*float top_line_len = distance(top_left, top_right);
	float right_line_len = distance(top_right, bot_right);
	float top_line_slope = get_slope(top_left, top_right);
	float right_line_slope = get_slope(top_right, bot_right);
	*/ 
	float right_diff = bot_right.x-top_right.x;
	float top_diff = top_left.y-top_right.y;

	VECTOR2 bot_left = {top_left.x+right_diff, bot_right.y+top_diff};
  }
  if (!br_set) {
	//std::cout << "Fill: Missing bottom right!" << std::endl;
 	/*float top_line_len = distance(top_left, top_right);
	float left_line_len = distance(top_left, bot_left);
	float top_line_slope = get_slope(top_left, top_right);
	float left_line_slope = get_slope(top_left, bot_left); 
	*/  

	float left_diff = bot_left.x-top_left.x;
	float top_diff = top_right.y-top_left.y;

	VECTOR2 bot_right = {top_right.x+left_diff, bot_left.y+top_diff};

  }

  clockwise_corners.push_back(top_left);
  clockwise_corners.push_back(top_right);
  clockwise_corners.push_back(bot_right);
  clockwise_corners.push_back(bot_left);
  
  gate_info->corner[0] = top_left;
  gate_info->corner[1] = top_right;
  gate_info->corner[2] = bot_right;
  gate_info->corner[3] = bot_left;
}

// Rearrange corners UL and clockwise
void GetClockwiseCorners(std::vector<VECTOR2>& clockwise_corners, GATE_INFO* gate_info) {
  VECTOR2 top_left, top_right, bot_left, bot_right;
  VECTOR2 midpoint = gate_info->midpoint;
  bool tl_set = false, tr_set = false, bl_set = false, br_set = false;
  
  for (int i = 0; i < 4; i++) {
    VECTOR2 cur_corner = gate_info->corner[i];
    if (cur_corner.x <= midpoint.x) {
	// left
	if (cur_corner.y <= midpoint.y) {
		//top
		top_left = cur_corner;
		tl_set = true;
	} else {
		// bot
		bot_left = cur_corner;
		bl_set = true;
	}
    } else {
	// right
	if (cur_corner.y <= midpoint.y) {
		// top
		top_right = cur_corner;
		tr_set = true;
	} else {
		bot_right = cur_corner;
		br_set = true;
	}
    }
  }

  if (!tl_set) {
	//std::cout << "Missing top left!" << std::endl;
  } else {
  	gate_info->corner[0] = top_left;
	clockwise_corners.push_back(top_left);
  }
  if (!tr_set) {
	//std::cout << "Missing top right!" << std::endl;
  } else {
  	gate_info->corner[1] = top_right;
	clockwise_corners.push_back(top_right);
  } 
  if (!br_set) {
	//std::cout << "Missing bottom right!" << std::endl;
  }
  else {
	gate_info->corner[2] = bot_right;
	clockwise_corners.push_back(bot_right);
  }
  if (!bl_set) {
	//std::cout << "Missing bottom left!" << std::endl;
  }
  else {
  	gate_info->corner[3] = bot_left;
	clockwise_corners.push_back(bot_left);
  }
}

void getGatePose(GATE_INFO *gate_info) {
  // char gates[11][10] = {"Gate10", "Gate21", "Gate2", "Gate13", "Gate9", "Gate14", "Gate1", "Gate22", "Gate15", "Gate23", "Gate6"};
  double ulx, uly, urx, ury, lrx, lry, llx, lly;

  for(int i = 0; i<4; i++){
    double corner_x = gate_info->corner[i].x, corner_y = gate_info->corner[i].y;
    if(corner_x < gate_info->midpoint.x && corner_y < gate_info->midpoint.y){
      ulx = corner_x;
      uly = corner_y;
    }else if(corner_x > gate_info->midpoint.x && corner_y < gate_info->midpoint.y){
      urx = corner_x;
      ury = corner_y;
    }else if(corner_x > gate_info->midpoint.x && corner_y > gate_info->midpoint.y){
      lrx = corner_x;
      lry = corner_y;
    }else if(corner_x < gate_info->midpoint.x && corner_y > gate_info->midpoint.y){
      llx = corner_x;
      lly = corner_y;
    }
  }


  cv::Mat reprojection(768, 1024, CV_8UC3, cv::Scalar(0,0,0));
    double gateSize = 2.0;
    std::vector<cv::Point3f> objPts;
    objPts.push_back(cv::Point3f(0,0,0));
    objPts.push_back(cv::Point3f(gateSize,0,0));
    objPts.push_back(cv::Point3f(0,gateSize,0));
    objPts.push_back(cv::Point3f(gateSize,gateSize,0));

    std::vector<cv::Point2f> imgPts;
    imgPts.push_back(cv::Point2f(llx,lly));
    imgPts.push_back(cv::Point2f(lrx,lry));
    imgPts.push_back(cv::Point2f(ulx,uly));
    imgPts.push_back(cv::Point2f(urx,ury));

    double fx = 548.4088134765625, cx = 512.0, fy = 548.4088134765625, cy = 384.0;
    double camera_mat_vec[9] = { fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0 };
    cv::Mat cameraMatrix = cv::Mat(3, 3, cv::DataType<double>::type, camera_mat_vec);

    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1);    // vector of distortion coefficients

    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);          // output rotation vector
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);          // output translation vector  

    cv::solvePnP(objPts, imgPts, cameraMatrix, distCoeffs, rvec, tvec);
    cv::Mat rotMatrix = cv::Mat::zeros(3, 3, CV_64FC1);   // rotation matrix
    Rodrigues(rvec, rotMatrix);
    // cv::Mat transform(4,4, CV_64FC1, rotMatrix, )

    std::vector<cv::Point2f> projPts;
    std::vector<cv::Point3f> axisPts;
    axisPts.push_back(cv::Point3f(0, 0, 0)); // Origin
    axisPts.push_back(cv::Point3f(0, 0, 1)); // Z-Axis
    axisPts.push_back(cv::Point3f(0, 1, 0)); // Y-Axis
    axisPts.push_back(cv::Point3f(1, 0, 0)); // X-Axis
    // axisPts.push_back(cv::Point3f(0.5, 0.5, 0)); // Mid-Point
    // axisPts.push_back(cv::Point3f(0.5, 0.5, 10)); // Extended Mid-Point
    cv::projectPoints(axisPts, rvec, tvec, cameraMatrix, distCoeffs, projPts);
    
    
      // for(unsigned int i = 0; i < projPts.size(); ++i)
      // {
      //   std::cout << "Image point: " << imgPts[i] << " Projected to " << projPts[i] << std::endl;
      // }

    // if(strcmp(marker.landmarkID.data.c_str(), "Gate10") == 0)std::cout << "M = "<< std::endl << " "  << cv::norm(tvec) << std::endl << std::endl;

    putText(reprojection, "UL", cv::Point(ulx, uly), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1);
    putText(reprojection, "UR", cv::Point(urx, ury), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1);
    putText(reprojection, "LR", cv::Point(lrx, lry), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1);
    putText(reprojection, "LL", cv::Point(llx, lly), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 1);

    cv::line(reprojection, cv::Point(ulx, uly), cv::Point(urx, ury), cv::Scalar(255,255,255), 1);
    cv::line(reprojection, cv::Point(urx, ury), cv::Point(lrx, lry), cv::Scalar(255,255,255), 1);
    cv::line(reprojection, cv::Point(lrx, lry), cv::Point(llx, lly), cv::Scalar(255,255,255), 1);
    cv::line(reprojection, cv::Point(llx, lly), cv::Point(ulx, uly), cv::Scalar(255,255,255), 1);

    cv::line(reprojection, cv::Point(projPts[0].x, projPts[0].y), cv::Point(projPts[1].x, projPts[1].y), cv::Scalar(128,255,255), 2);
    cv::line(reprojection, cv::Point(projPts[0].x, projPts[0].y), cv::Point(projPts[2].x, projPts[2].y), cv::Scalar(255,128,255), 2);
    cv::line(reprojection, cv::Point(projPts[0].x, projPts[0].y), cv::Point(projPts[3].x, projPts[3].y), cv::Scalar(255,255,128), 2);
    // cv::line(reprojection, cv::Point(projPts[4].x, projPts[4].y), cv::Point(projPts[5].x, projPts[5].y), cv::Scalar(255,255,128), 2);

  // }
    
  // cv::hconcat(rotMatrix, rotMatrix);
  cv::Mat H, V;
  cv::hconcat(rotMatrix, tvec, H);
  double data[10] = { 0, 0, 0, 1 };
  cv::Mat Z = cv::Mat(1, 4, CV_64FC1, data);
  cv::vconcat(H, Z, V);
  cv::Mat trans = V.inv();

  // imshow("Reprojection", reprojection);cv::waitKey(1);
  gate_info->transform = trans;
  // std::cout << "Z:" << gate_info->transform.at<double>(0,3) 
  //           << "\nY:" << gate_info->transform.at<double>(1,3) 
  //           << "\nX:" << gate_info->transform.at<double>(2,3) 
  //           << std::endl;

  // Save drone position w.r.t Gate

  double drone_x = gate_info->transform.at<double>(0,3); 
  double drone_y = gate_info->transform.at<double>(1,3);
  double drone_z = gate_info->transform.at<double>(2,3);
  saveGatePoseToBuffer(drone_x, drone_y, drone_z);
}

void gateInfo(GATE_INFO *gate_info) {
  if(!ir_markersVec.size())
    return;
  for (int i = 0; i < 4; i++)
    gate_info->corner[i] = {0, 0};
  gate_info->midpoint = {0, 0};
  int n_corners = 0;
  sprintf(buffer, "Gate%d", gate_sequence[gate_info->target_index]);
  gate_info->found = false;
  

  for (int i = 0; i < ir_markersVec.back().markers.size(); i++) {
    if (strcmp(ir_markersVec.back().markers[i].landmarkID.data.c_str(), buffer) == 0) {
      int k = atoi(ir_markersVec.back().markers[i].markerID.data.c_str()) - 1;
      gate_info->midpoint.x += gate_info->corner[k].x = ir_markersVec.back().markers[i].x;
      gate_info->midpoint.y += gate_info->corner[k].y = ir_markersVec.back().markers[i].y;
      n_corners++;
      gate_info->found = true;
    }
  }
  
  
  if (n_corners) {
    gate_info->midpoint.x /= n_corners;
    gate_info->midpoint.y /= n_corners;
  }
  std::vector<VECTOR2> clockwise_corners;
  if (n_corners == 4) {
    //std::cout << "n corners is " << n_corners << std::endl;
    GetClockwiseCorners(clockwise_corners, gate_info);
  }
  else if (n_corners == 3) {
    //std::cout << "n corners is " << n_corners << std::endl;
    FillAndGetClockwiseCorners(clockwise_corners, gate_info);
  }

  if (n_corners >= 3) { // Last Good Midpoint
    gate_info->lgm = gate_info->midpoint;
    for (int i = 0; i < 4; i++)
      gate_info->lgc[i] = gate_info->corner[i];
  }
 
  gate_info->length = 0;
  int n_sides = 0;
  if (gate_info->corner[0].x && gate_info->corner[3].x) {
    gate_info->length = distance(gate_info->corner[0], gate_info->corner[3]);
    n_sides++;
  }
  if (gate_info->corner[1].x && gate_info->corner[2].x) {
    gate_info->length += distance(gate_info->corner[1], gate_info->corner[2]);
    n_sides++;
  }
  if ((n_sides == 0) && gate_info->corner[2].x && gate_info->corner[3].x) {
  	// Only use bottom edge if neither side is seen.
    gate_info->length += distance(gate_info->corner[2], gate_info->corner[3]);
    n_sides++;
  }
  gate_info->length = n_sides ? gate_info->length / n_sides : -1;
  gate_info->distance = 700 / gate_info->length;

  //if (gate_sequence[gate_target] == 23)
    //ROS_INFO("Distance = %f, Length = %f (%f) lgm.x = %f nc = %d",
    //  gate_info->distance, gate_info->length, prev_length, gate_info->lgm.x, n_corners);
  
  //   last Good Distance : Sometimes, distance becomes negative. Discard it.
  if( gate_info->distance >= 0){
     lastGoodDistance = gate_info->distance;
  } 
 
  //ROS_INFO("Distance = %f, Length = %f", gate_info->distance, gate_info->length);

  gate_info->passed = ((gate_info->length == -1) && (prev_length != -1) 
      && (prev_length > 200)); // && (abs(screen.x/2 - gate_info->lgm.x) < screen.x/4.3));
  // Gate 2 & 6 hack
  if (((gate_info->target_index == 1) || (gate_info->target_index == 5))
  		&& (n_corners <= 2) && (prev_length > 200)) {
  	gate_info->passed = true;
    ROS_INFO("HACK PASSED \a");
  }
 
  //Recalculate the centerpoint 
  double centerpoint = screen.x/2 - fudgefactor[gate_target];

  // Make sure we didn't skirt the edge
  if (((gate_info->lgc[0].x > 1) && (gate_info->lgc[0].x > centerpoint)) || 
      ((gate_info->lgc[3].x > 1) && (gate_info->lgc[3].x > centerpoint)) ||
      ((gate_info->lgc[1].x > 1) && (gate_info->lgc[1].x < centerpoint)) ||
      ((gate_info->lgc[2].x > 1) && (gate_info->lgc[2].x < centerpoint))) {
    if (gate_info->passed)
      ROS_INFO("Skirts!  %f %f %f %f %f", screen.x / 2,
        gate_info->lgc[0].x, gate_info->lgc[1].x, gate_info->lgc[2].x, gate_info->lgc[3].x);
    gate_info->passed = false;
  }

  prev_length = gate_info->length;

}


void dumpStats() {
	std::ofstream file;
  	file.open ("/tmp/run_stats_"+timestamp+".log");
	
	file << "Num runs: " << num_runs << "\n";
	
	for(int i = 0; i < gate_names.size(); i++) {
		file << "Num " << gate_names[i] << " passes: " << stats[gate_names[i]] << "\n";
	}
	file.close();
	
}


// Follow S curve when needed
double sigmoid(double x, double maximum, double inflection) {
  return maximum / (1 + exp(inflection - x));
}

GATE_INFO gate_info;

void pilotUpdate(void) {
  if(!ir_markersVec.size())
    return;
  if (collision_flag) {
    //dumpStats();
    collision_flag = false;
    num_runs++;
    num_passed_gates = 0;
    gate_target = 0;
    gate_state = GATE_LOOKING;
    wait_timer = 20;   // Their collision callback needs some debouncing.  2 seconds works ok.
    rate_thrust.angular_rates.x = 0;
    rate_thrust.angular_rates.y = 0;
    rate_thrust.angular_rates.z = 0;
    rate_thrust.thrust.z        = 0;
    return;
  }
  if (wait_timer) {
    wait_timer--;
    return;
  }

  do {
    gate_info.target_index = gate_target;
    gateInfo(&gate_info);

    //ROS_INFO("Velocity %f towards Gate %d.", getDroneVelocity(), gate_target);

    getGatePose(&gate_info);
    if ((gate_state == GATE_LOOKING) && (gate_info.found)) {
      gate_state = GATE_FOUND;
      ROS_INFO("Found Gate %d.", gate_target);
    } else if ((gate_state == GATE_FOUND) && (gate_info.passed)) {
      ROS_INFO("Passed Gate %d.", gate_target);
      num_passed_gates++;
      //stats[gate_names[gate_target]]++;
      if (++gate_target >= NUM_GATES) {
        gate_state = RACE_FINISHED;
        ROS_INFO("RACE_FINISHED!!!!!:%d", gate_target);
      } else {
        gate_state = GATE_LOOKING;
        ROS_INFO("Looking for Gate %d.", gate_target);
      }
    } else if ((gate_state == GATE_FOUND) && (!gate_info.found)) {
      gate_state = GATE_LOOKING;
      ROS_INFO("Lost Gate %d %f %f.", gate_target, gate_info.length, prev_length);
    }
  } while (gate_info.passed);

  // Set the target point
  VECTOR2 target_point = {screen.x/2, screen.y/2};
  // Move over tweaks from RL/GA model
  if (gate_target == 2) target_point.x += 50;
  if (gate_sequence[gate_target] == 9) {target_point.x += 100;}
  if (gate_target == 0) {target_point.x -= 6;target_point.y -= 95;}
  if (gate_target == 1) target_point.y -= 100;
  if (gate_sequence[gate_target] == 1) target_point.x += 100; // gate_target 6
  if (gate_sequence[gate_target] == 13) {target_point.x -= 80;target_point.y -= 100;}
  if (gate_sequence[gate_target] == 23) target_point.x -= 80;
  if (gate_target == 5) {target_point.x +=40;target_point.y -= 130;}
  if (gate_sequence[gate_target] == 22) target_point.x += 100; // gate_target 7
  if (gate_target == 8) target_point.x += 100;
  if (gate_target == 9) target_point.x -= 100;
  if (gate_target == 10) target_point.x -= 100;

  // Set our control values
  double roll, pitch, yaw, thrust;
  if (gate_state == GATE_FOUND) {
    // Initial learned settings
    yaw = lrnY[gate_target] * (target_point.x - gate_info.midpoint.x) / screen.x;

    // Move over tweaks from RL/GA model
    if (gate_sequence[gate_target] == 13) {
        yaw += sigmoid(gate_info.distance, 1.0, 5);
    }
    if (gate_sequence[gate_target] == 9) {
        yaw -= sigmoid(gate_info.distance, 2.0, 6);
    }
	  if (gate_target == 6) yaw += 0.5;
    if (gate_sequence[gate_target] == 22) {
      yaw += 0.75;
    }
    if (gate_sequence[gate_target] == 23) {
      yaw -= 0.5;
    }
	  if (gate_sequence[gate_target] == 15) {
      yaw += 1.0;
    }

    // Default
    roll = -yaw/2;

    // Move over tweaks from RL/GA model
    if (gate_sequence[gate_target] == 22) {
      roll = -yaw;
    }

    pitch = 0.1 - 0.5 * (target_point.y - gate_info.midpoint.y) / screen.y;
    pitch = std::max(pitch, -0.2);

    // Initial learned settings
    thrust = 9.9 + 1 * (target_point.y - gate_info.midpoint.y) / screen.y;

    // Move over tweaks from RL/GA model
    if (gate_target == 0) {thrust += 0.4;}
    if (gate_target == 1) {thrust += 1.0;}
    if (gate_target == 2) {thrust += 0.3;};
    if (gate_target == 3) thrust += 1.3;
    if (gate_sequence[gate_target] == 9) thrust += 0.3;
    if (gate_sequence[gate_target] == 23) thrust += 0.1;
    if (gate_sequence[gate_target] == 6) thrust += 1.4;
    if (gate_target == 6) thrust += 0.5;
    if (gate_target == 7) thrust += 1;
    if (gate_target == 8) thrust += 0.6;
    if (gate_target == 9) thrust -= 0.5; //0.0;
    if (gate_target == 10) thrust += 1;
  } else {
    // Default suggestion when gate is not found
  	roll = gate_looking_default[gate_target].r;
  	pitch = gate_looking_default[gate_target].p;
  	yaw = gate_looking_default[gate_target].y;
  	thrust = gate_looking_default[gate_target].t;
  }

  rate_thrust.angular_rates.x = roll;   	// Roll: +right
  rate_thrust.angular_rates.y = pitch;	// Pitch:  +forward
  rate_thrust.angular_rates.z = yaw;	  // Yaw: +left
  rate_thrust.thrust.z        = thrust;	  // Thrust +up
  //ROS_INFO("%f %f %f %f", rate_thrust.angular_rates.x , rate_thrust.angular_rates.y, rate_thrust.angular_rates.z, rate_thrust.thrust.z);
  //ROS_INFO("Imu: %f %f %f", imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z);
  //if (gate_sequence[gate_target] == 9) {
  //  ROS_INFO("Gate9 %s l = %f d = %f tp = %f %f y = %f", gate_info.found ? "F" : ".",
  //    gate_info.length, gate_info.distance, target_point.x, gate_info.midpoint.x, yaw);
  //}
}
