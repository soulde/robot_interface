#include "RobotInterface.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "serial");
    ros::NodeHandle nh;
    RobotInterface dealer(nh);
    while(ros::ok()){
        ros::spinOnce();
    }
}