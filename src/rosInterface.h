#include "mycube.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "rosrt/rosrt.h"
class Marker_getter {
  public:
    virtual bool update(marker &get_marker)=0;
};

class rosInterface: public Marker_getter{
  public: 
  rosInterface(char*);
  bool update(marker &myMarker);
private:
  visualization_msgs::MarkerConstPtr msg;
  ros::NodeHandle *handle;
  rosrt::Subscriber<visualization_msgs::Marker> *subscriber;
};

