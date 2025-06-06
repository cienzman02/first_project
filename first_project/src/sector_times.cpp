#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include "first_project/sector_times.h"

#include <cmath>
#include <vector>
#include <iostream>
#include <algorithm>


/*

tra settore 3-1: (45.616058, 9,280528) -- (45.616018, 9.281218)
tra settore 1-2: (45.630247, 9.289481) -- (45.629969, 9.289499)
tra settore 2-3: (45.623658, 9.287174) -- (45.623476, 9.287370)

*/

struct Coordinate {
    double latitude;
    double longitude;
};


std::vector<std::pair<Coordinate, Coordinate>> sectors = {
    // Settore 1-2
    { {45.630247, 9.289481}, {45.629969, 9.289499} },

    // Settore 2-3
    { {45.623658, 9.287174}, {45.623476, 9.287370} },
    
    // Settore 3-1
    { {45.616058, 9.280528}, {45.616018, 9.281218} },
};

double latest_lat = 0.0, latest_lon = 0.0;    // coordinate GPS
bool received_gps = false;                    // per sapere se abbiamo già ricevuto almeno un messaggio
int sector_number = 0;
Coordinate latest_coordinate;

ros::Publisher sec_times_pub;
float mean_speed = 0.0;
size_t n_speed_samples = 0;
double sec_start_time;
bool sec_time_initialized = false;

int orientation(Coordinate p, Coordinate q, Coordinate r) {
    double val = (q.latitude - p.latitude) * (r.longitude - q.longitude) - 
                 (q.longitude - p.longitude) * (r.latitude - q.latitude);
    if (fabs(val) < 1e-9) return 0;  // collineari
    return (val > 0) ? 1 : 2;  // orario o antiorario
}

// Verifica se il punto q si trova sul segmento pr
bool onSegment(Coordinate p, Coordinate q, Coordinate r) {
    return (q.longitude <= std::max(p.longitude, r.longitude) && 
            q.longitude >= std::min(p.longitude, r.longitude) &&
            q.latitude <= std::max(p.latitude, r.latitude) && 
            q.latitude >= std::min(p.latitude, r.latitude));
}

// Verifica se due segmenti si intersecano
bool doSegmentsIntersect(Coordinate p1, Coordinate p2, Coordinate p3, Coordinate p4) {
    // Calcola i quattro orientamenti necessari
    int o1 = orientation(p1, p2, p3);
    int o2 = orientation(p1, p2, p4);
    int o3 = orientation(p3, p4, p1);
    int o4 = orientation(p3, p4, p2);

    // Caso generale
    if (o1 != o2 && o3 != o4)
        return true;

    // Casi speciali (quando i punti sono collineari)
    if (o1 == 0 && onSegment(p1, p3, p2)) return true;
    if (o2 == 0 && onSegment(p1, p4, p2)) return true;
    if (o3 == 0 && onSegment(p3, p1, p4)) return true;
    if (o4 == 0 && onSegment(p3, p2, p4)) return true;

    return false;
}

// Callback for /speedsteer
void speedCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    // ROS_INFO("Speed: %.6f", msg->point.y);
    mean_speed = (mean_speed * (float)n_speed_samples + msg->point.y) / ((float)n_speed_samples + 1.);
    n_speed_samples++;
}

// Callback for /swiftnav/front/gps_pose
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    if (!sec_time_initialized) {
        sec_start_time = msg->header.stamp.toSec();
        sec_time_initialized = true;
    }

    Coordinate new_coordinate;
    new_coordinate.latitude = msg->latitude;
    new_coordinate.longitude = msg->longitude;

    Coordinate A = sectors[sector_number].first;
    Coordinate B = sectors[sector_number].second;

    if (doSegmentsIntersect(A, B, latest_coordinate, new_coordinate) && received_gps) {
        ROS_INFO("sector_times: Finished sector %d, going to sector %ld, transition at lat=%.8f, lon=%.8f", 
            sector_number + 1, (sector_number + 1) % sectors.size() + 1, msg->latitude, msg->longitude);
        sector_number = (sector_number + 1) % sectors.size();
        mean_speed = 0.0;
        n_speed_samples = 0;
        sec_start_time = msg->header.stamp.toSec();
    }

    latest_coordinate = new_coordinate;
    received_gps = true;
    //ROS_INFO("GPS received");
    //ROS_INFO("Current sector: %d", sector_number);

    first_project::sector_times sec_msg;
    sec_msg.current_sector = sector_number + 1;
    sec_msg.current_sector_time = msg->header.stamp.toSec() - sec_start_time;
    sec_msg.current_sector_mean_speed = mean_speed;
    sec_times_pub.publish(sec_msg);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "sector_times");
    ros::NodeHandle nh;
    
    ros::Subscriber sub_speed = nh.subscribe("/speedsteer", 10, speedCallback);
    ros::Subscriber sub_gps = nh.subscribe("/swiftnav/front/gps_pose", 10, gpsCallback);
    sec_times_pub = nh.advertise<first_project::sector_times>("/sector_times", 50);

    ROS_INFO("sector_times: Odometer node started and subscribed to /speedsteer and /swiftnav/front/gps_pose");

    ros::spin();
    return 0;
}
