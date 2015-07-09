#include "rosInterface.h"


  rosInterface::rosInterface(char *topic)
  {
 
  // put in the beginning of the task ?
  #ifdef __XENO__
    mlockall(MCL_CURRENT | MCL_FUTURE);
  #endif
    
    int argc = 1;
    char name[] = "SL";
    char* argv[1];
    argv[0] = name;
    
    ros::init(argc,argv,"ros_rt_test_subscriber");
    
    
    struct rosrt::InitOptions options;
    options.pubmanager_thread_name = "ros_rt_test_subscriber";
    
    handle = new ros::NodeHandle();
    
    rosrt::init(options);
    subscriber = new rosrt::Subscriber<visualization_msgs::Marker>(3, *handle, topic);
  }
  bool rosInterface::update(struct marker &myMarker)
  {
    if (ros::ok()){
      msg = subscriber->poll();
      if (msg)
      {
        myMarker.x = msg->pose.position.x;
        myMarker.y = msg->pose.position.y;
        myMarker.z = msg->pose.position.z;
        myMarker.xq = msg->pose.orientation.x;
        myMarker.yq = msg->pose.orientation.y;
        myMarker.zq = msg->pose.orientation.z;
        myMarker.wq = msg->pose.orientation.w;
        myMarker.id = msg->id;
        return true;
      }
      return false;
    }
  }
