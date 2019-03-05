#include "../../include/rdda_interface/rdda_node.h"

using namespace std;

/* RDDNode constructor */
RDDNode::RDDNode(ros::NodeHandle &node, shared_in_t *in, shared_out_t *out) {
    nh_ = node;
    shared_in = in;
    shared_out = out;
    target_position_sub = nh_.subscribe("/rdd/set_position", 1, &RDDNode::setPositionCallback, this);
    actual_position_pub = nh_.advertise<std_msgs::Float64>("/rdd/actual_position", 1);
}

RDDNode::~RDDNode() {};

/* Publish joint state */
void RDDNode::publishJointState() {
    std_msgs::Float64 position_msg;
    ticket_lock(&shared_out->queue);
    position_msg.data = shared_out->act_pos; /* Publish actual position */
    ticket_unlock(&shared_out->queue);
    ROS_INFO("Get actual position %lf", position_msg.data);
    actual_position_pub.publish(position_msg);
}

/* Subscriber callback */
void RDDNode::setPositionCallback(const std_msgs::Float64ConstPtr& msg) {
    ticket_lock(&shared_in->queue);
    shared_in->tg_pos = msg->data;
    ticket_unlock(&shared_in->queue);
    ROS_INFO("set target position: %lf", msg->data);
}

/* Run loop */
void RDDNode::run() {
    ros::Rate loop_rate(10);
    while (ros::ok()) {
	/* Publisher (wrap) */
	publishJointState();
	/* Subscriber callback loop */
	ros::spinOnce();
	loop_rate.sleep();
    }
}

/* Initialise shared memory and run */
int main(int argc, char** argv) {

    /* Local variables */
    void *p;	/* Intermediate pointer */
    // int err; 	/* Error number */
    int i;	 	/* Loop iterations */
    double pos = 0.0; 

    /* Instanciate input-output data varibles */
    shared_in_t *shared_in;
    shared_out_t *shared_out;

    /* Map data structs to shared memory */
    /* Open and obtain shared memory pointers for master-input data */
    if (!openSharedMemory(SHARED_IN, &p)) {
	shared_in = (shared_in_t *) p;
    } else {
	fprintf(stderr, "Open(shared_in)\n");
	return -1;
    }

    /* Initialise ticket lock */
    ticket_init(&shared_in->queue);

    /* Open and obtain shared memory pointers for master-output data */
    if (!openSharedMemory(SHARED_OUT, &p)) {
	shared_out = (shared_out_t *) p;
    } else {
	fprintf(stderr, "Open(shared_out)\n");
	return -1;
    }

    /* Initialise ticket lock */
    ticket_init(&shared_out->queue);

    /* Initialise ROS node */
    ros::init(argc, argv, "rdd");
    printf("Launch ros interface\n");

    ros::NodeHandle node("~");
    RDDNode rdd(node, shared_in, shared_out);
    rdd.run();

}
