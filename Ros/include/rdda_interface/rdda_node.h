#ifndef RDDA_INTERFACE_H
#define RDDA_INTERFACE_H

/* C++ headers */
#include <pthread.h>

/* ROS headers */
#include <ros/ros.h>
#include <std_msgs/Float64.h>

/* C headers */
extern "C" {
#include "./shared_data.h" 
#include "./shared_memory.h"
};

class RDDNode {
 public:
    explicit RDDNode(ros::NodeHandle& node, shared_in_t *in, shared_out_t *out);

    ~RDDNode();

    void run();

 private:
    ros::NodeHandle nh_;
    ros::Subscriber target_position_sub;
    ros::Publisher actual_position_pub;

    shared_in_t *shared_in;
    shared_out_t *shared_out;

    void publishJointState();
    void setPositionCallback(const std_msgs::Float64ConstPtr& msg);
};

#endif /* RDDA_INTERFACE_H */
