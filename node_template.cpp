#include "ros/ros.h"
#include "std_msgs/String.h"


class MyClass{
    ros::Subscriber data_sub;
    ros::Publisher data_pub;
    double parameter1;

public:
    MyClass(){
        ros::NodeHandle n;
        ros::NodeHandle np("~");
        parameter1 = 0.5
        np.getParam("parameter1", parameter1);
        data_pub = n.advertise<std_msgs::String>("data_out",10);
        data_sub = n.subscribe("data",10,&MyClass::dataCB,this);
    }

private:
    void dataCB(const std_msgs::String::ConstPtr &msg){
        data_pub.publish(msg);
    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "node_name");
    MyClass mc;
    ros::spin();
    return 0;
}
