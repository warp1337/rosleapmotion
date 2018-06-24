#include <string.h>
#include <boost/shared_ptr.hpp>
#include <sstream>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "camera_info_manager/camera_info_manager.h"
#include "rospack/rospack.h"
#include "Leap.h"


#define targetWidth 500
#define targetHeight 500
#define cutWidth 280
#define cutHeight 220
#define startX 110
#define startY 140

using namespace Leap;
using namespace std;

class CameraListener : public Listener {
  public:
    boost::shared_ptr <ros::NodeHandle> _left_node;
    boost::shared_ptr <ros::NodeHandle> _right_node;

    ros::Publisher _pub_image_left;
    ros::Publisher _pub_info_left;
    ros::Publisher _pub_image_right;
    ros::Publisher _pub_info_right;
    camera_info_manager::CameraInfoManager* info_mgr_right;
    camera_info_manager::CameraInfoManager* info_mgr_left;
    
    bool enable_controller_info = false;

    unsigned int seq;
    virtual void onInit(const Controller&);
    virtual void onConnect(const Controller&);
    virtual void onDisconnect(const Controller&);
    virtual void onExit(const Controller&);
    virtual void onFrame(const Controller&);
    virtual void onFocusGained(const Controller&);
    virtual void onFocusLost(const Controller&);
    virtual void onDeviceChange(const Controller&);
    virtual void onServiceConnect(const Controller&);
    virtual void onServiceDisconnect(const Controller&);  
};


void CameraListener::onInit(const Controller& controller){

    _left_node = boost::make_shared<ros::NodeHandle>("left");
    _right_node = boost::make_shared<ros::NodeHandle>("right");

    _pub_image_left = _left_node->advertise<sensor_msgs::Image>("image_raw", 1);
    _pub_image_right = _right_node->advertise<sensor_msgs::Image>("image_raw", 1);

    _pub_info_left = _left_node->advertise<sensor_msgs::CameraInfo>("camera_info", 1);
    _pub_info_right = _right_node->advertise<sensor_msgs::CameraInfo>("camera_info", 1);
    
    seq = 0;
    rospack::Rospack rp;
    std::string path;
    std::string default_l_info_filename;
    std::string default_r_info_filename;
    std::vector<std::string> search_path;

    rp.getSearchPathFromEnv(search_path);
    rp.crawl(search_path, 1);

    if ( rp.find("leap_motion",path) == true) 
    {
        default_l_info_filename = path + std::string("/config/camera_info/leap_cal_left.yml");
        default_r_info_filename = path + std::string("/config/camera_info/leap_cal_right.yml");
    }
    else
    {
        default_l_info_filename = "";
        default_r_info_filename = "";
    }

    ros::NodeHandle local_nh("~");
    std::string l_info_filename;
    std::string r_info_filename;

    local_nh.param("template_filename_left", l_info_filename, default_l_info_filename);
    local_nh.param("template_filename_right", r_info_filename, default_r_info_filename);
    
    l_info_filename = std::string("file://") + l_info_filename;
    r_info_filename = std::string("file://") + r_info_filename;
    
    info_mgr_left = new camera_info_manager::CameraInfoManager(*_left_node, "left", l_info_filename);
    info_mgr_right = new camera_info_manager::CameraInfoManager(*_right_node, "right", r_info_filename);

    if(CameraListener::enable_controller_info)
    {  
        ROS_INFO("CameraListener initialized");
    }
}

void CameraListener::onConnect(const Controller& controller) {
    if(CameraListener::enable_controller_info)
    {  
        ROS_INFO("CameraListener connected");
    }
}

void CameraListener::onDisconnect(const Controller& controller) {
    if(CameraListener::enable_controller_info)
    {
        ROS_INFO("CameraListener disconnected");
    }
}

void CameraListener::onExit(const Controller& controller) {
    if(CameraListener::enable_controller_info)
    {  
        ROS_INFO("CameraListener exited");
    }
}

void CameraListener::onFrame(const Controller& controller) {
  // Get the most recent frame and report some basic information
  const Frame frame = controller.frame();  
  // The list of IR images from the Leap Motion cameras. 
  ImageList images = frame.images();
  // http://docs.ros.org/api/sensor_msgs/html/msg/Image.html
  sensor_msgs::Image image_msg;

  image_msg.header.seq = seq++;
  image_msg.header.stamp = ros::Time::now();
  // Encoding of pixels -- channel meaning, ordering, size
  // taken from the list of strings in include/sensor_msgs/image_encodings.h
  image_msg.encoding = "mono8";
  image_msg.is_bigendian = 0;
  
  for(int camera_num = 0; camera_num < 2; camera_num++){
    Image image = images[camera_num];
    
    // image width, that is, number of columns
    image_msg.width =  cutWidth;
    // image height, that is, number of rows
    image_msg.height = cutHeight;
    // Full row length in bytes
    image_msg.step = cutWidth;
    image_msg.data.resize(cutWidth * cutHeight);

    // The parallel construct forms a team of threads and starts parallel execution.
    // The loop construct specifies that the iterations of loops will be distributed 
    // among and executed by the encountering team of threads.
    #pragma omp parallel for
    
    for(int i = 0; i < cutWidth; i++)
    {
      for(int j = 0; j < cutHeight; j++)
      {
        Vector input = Vector((float)(i + startX)/targetWidth, (float)(j + startY)/targetHeight, 0);
        input.x = (input.x - image.rayOffsetX()) / image.rayScaleX();
        input.y = (input.y - image.rayOffsetY()) / image.rayScaleY();

        Vector pixel = image.warp(input);
        if(pixel.x >= 0 && pixel.x < image.width() && pixel.y >= 0 && pixel.y < image.height()) 
        {
          int data_index = floor(pixel.y) * image.width() + floor(pixel.x);
          image_msg.data[cutWidth*j+i] = image.data()[data_index];
        } 
        else
          image_msg.data[cutWidth*j+i] = 0;
      }
    }

    if(camera_num == 0)
    {
      sensor_msgs::CameraInfoPtr info_msg(new sensor_msgs::CameraInfo(info_mgr_left -> getCameraInfo() ) );
      
      image_msg.header.frame_id = info_msg->header.frame_id ="leap_pointcloud";
      info_msg->width = image_msg.width;
      info_msg->height = image_msg.height;
      info_msg->header.stamp = image_msg.header.stamp;
      info_msg->header.seq = image_msg.header.seq;
      _pub_image_left.publish(image_msg);
      _pub_info_left.publish(*info_msg);
    }
    else
    {
      sensor_msgs::CameraInfoPtr info_msg(new sensor_msgs::CameraInfo(info_mgr_right->getCameraInfo()));
      image_msg.header.frame_id = info_msg->header.frame_id = "lmc_";
      info_msg->width = image_msg.width;
      info_msg->height = image_msg.height;
      info_msg->header.stamp = image_msg.header.stamp;
      info_msg->header.seq = image_msg.header.seq;
      _pub_image_right.publish(image_msg);
      _pub_info_right.publish(*info_msg);
    }
  }
}

void CameraListener::onFocusGained(const Controller& controller) {
    if(CameraListener::enable_controller_info)
    {  
        ROS_INFO("CameraListener gained focus");
    }
}

void CameraListener::onFocusLost(const Controller& controller) {
    if(CameraListener::enable_controller_info)
    {  
        ROS_INFO("CameraListener lost focus");
    }
}

void CameraListener::onDeviceChange(const Controller& controller) {
    if(CameraListener::enable_controller_info)
    {  
        ROS_INFO("CameraListener device changed");
        const DeviceList devices = controller.devices();
        for (int i = 0; i < devices.count(); ++i) {
            ROS_INFO( "id: %s", devices[i].toString().c_str() );
            ROS_INFO("  isStreaming: %s", (devices[i].isStreaming() ? "true" : "false") );
        }
    }
}

void CameraListener::onServiceConnect(const Controller& controller) {
    if(CameraListener::enable_controller_info)
    {
        ROS_INFO("CameraListener service connected");
    }
}

void CameraListener::onServiceDisconnect(const Controller& controller) {
    if(CameraListener::enable_controller_info)
    {
        ROS_INFO("CameraListener service disconnected");
    }
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "leap_motion");
  
  CameraListener listener;
  Controller controller;
  controller.addListener(listener);
  controller.setPolicyFlags(static_cast<Leap::Controller::PolicyFlag> (Leap::Controller::POLICY_IMAGES));
  
  ros::spin();
  controller.removeListener(listener);

  return 0;
}
