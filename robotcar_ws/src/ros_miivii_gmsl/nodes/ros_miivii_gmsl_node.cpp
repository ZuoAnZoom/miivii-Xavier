#include <ros/ros.h>
#include "MiiViiCamSDK.h"
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sstream>
#include <std_srvs/Empty.h>
#include <unistd.h>
#include <sensor_msgs/fill_image.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <cv_bridge/cv_bridge.h>
#include <signal.h>
#include <sys/stat.h>
#define TEST 0
#define TEST_TIMESTAMP 0

#define MAX_CAMERA_NUM 4
using namespace std;
namespace ros_miivii_gmsl {

class ros_miivii_gmsl_Node
{
public:
  // private ROS node handle
  ros::NodeHandle node_;

  std::string camera_name_, camera_info_url_;
  std::string devName;

  uint camWidth, camHeight, fps,  camCount;
  int  camWidth_, camHeight_, fps_,  camCount_;

  MVGmslCamSDK *m_gmsl_camera;
  uint8_t *outbuf[MAX_CAMERA_NUM];

  int   enable_sync_;
  bool bEnable_sync;//true R5时间;false ros time now


  // shared image message
  sensor_msgs::Image img_msg[MAX_CAMERA_NUM];

  sensor_msgs::Image img_with_deviceTime[MAX_CAMERA_NUM];

  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_[MAX_CAMERA_NUM];

  image_transport::CameraPublisher image_pub_[MAX_CAMERA_NUM];

#if TEST_TIMESTAMP
  sensor_msgs::Image img_msg_RosTime[MAX_CAMERA_NUM];
  image_transport::CameraPublisher image_pub_RosTime[MAX_CAMERA_NUM];
#endif

  ros_miivii_gmsl_Node() :
      node_("~")
  {
    ROS_INFO("starting to run construct function");

    // grab the parameters
    node_.param("video_device", devName, std::string("/dev/video0"));
    node_.param("image_height", camHeight_, 728);
    camHeight =camHeight_;
    node_.param("image_width", camWidth_, 1280);
    camWidth = camWidth_;

    node_.param("camCount", camCount_, 1);
    camCount = camCount_;
    node_.param("fps", fps_, 25);
    fps = fps_;

    node_.param("enable_sync", enable_sync_, 1);
    if(enable_sync_==1)//用设备时间
    {
      bEnable_sync = true;
      ROS_INFO("Enable sync");
    }
    else//用ros time时间
    {
      bEnable_sync = false;
    }

    ROS_INFO("devName = %s   camWidth = %d   camHeight = %d    fps = %d",devName.c_str(),camWidth, camHeight,fps);
    m_gmsl_camera = new MVGmslCamSDK(devName, camCount, camWidth, camHeight,fps);
    //zhh

    node_.param("camera_name", camera_name_, std::string("gmsl_camera"));

    node_.param("camera_info_url", camera_info_url_, std::string(""));

    // advertise the main image topic
    image_transport::ImageTransport it(node_);

    for(int i=0;i<camCount;i++)
    {
      image_pub_[i] = it.advertiseCamera("image_gmsl_raw" + std::to_string(i), 1);

      cinfo_[i].reset(new camera_info_manager::CameraInfoManager(node_, camera_name_+ std::to_string(i), camera_info_url_));

      img_msg[i].header.frame_id = std::string("gmsl_camera")+std::to_string(i);
#if TEST_TIMESTAMP
      image_pub_RosTime[i] = it.advertiseCamera("image_gmsl_raw0" + std::to_string(i), 1);
      img_msg_RosTime[i].header.frame_id = std::string("gmsl_camera0")+std::to_string(i);
#endif
      //if (!cinfo_[i]->isCalibrated())
      {
        cinfo_[i]->setCameraName(devName+std::to_string(i));
        sensor_msgs::CameraInfo camera_info;
        camera_info.header.frame_id = img_msg[i].header.frame_id;
        camera_info.width  = camWidth;
        camera_info.height = camHeight;
        cinfo_[i]->setCameraInfo(camera_info);
      }
    }
    for(int i=0;i<MAX_CAMERA_NUM;i++)
    {
      outbuf[i] =NULL;
    }
  }

  virtual ~ros_miivii_gmsl_Node()
  {
    if(m_gmsl_camera!=NULL)
    {
      delete m_gmsl_camera;
      m_gmsl_camera = NULL;
    }
  }

  bool grab_and_send_image()
  {
    uint64_t timestap;
    if (m_gmsl_camera->MVGmslGetImage(outbuf, timestap))//if (m_gmsl_camera->MVGmslGetImage(outbuf,timestap))
    {
      // grab the camera ROS_INFO
      ros::Time  Ts;
      if(bEnable_sync)
      {
        Ts.fromNSec(timestap);
      }
      else
      {
        Ts = ros::Time::now();
      }

      for(int i=0;i<camCount; i++)
      {
        img_msg[i].header.stamp = Ts;

#if TEST
        cv::Mat imgbuf = cv::Mat(camHeight, camWidth, CV_8UC4, outbuf[i]);
        fillImage(img_msg[i], "rgba8", camHeight, camWidth, 4 * camWidth, imgbuf.data);
#else
        fillImage(img_msg[i], "rgba8", camHeight, camWidth, 4 * camWidth, outbuf[i]);
#endif

        sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_[i]->getCameraInfo()));

        ci->header.frame_id = img_msg[i].header.frame_id;
        ci->header.stamp    = img_msg[i].header.stamp;

        // publish the image
        image_pub_[i].publish(img_msg[i], *ci);
      }


#if TEST_TIMESTAMP
      for(int i=0;i<camCount; i++)
      {
        img_msg_RosTime[i].header.stamp = Ts;

        fillImage(img_msg_RosTime[i], "rgba8", camHeight, camWidth, 4 * camWidth, outbuf[i]);

        sensor_msgs::CameraInfoPtr ci1(new sensor_msgs::CameraInfo(cinfo_[i]->getCameraInfo()));

        ci1->header.frame_id = img_msg_RosTime[i].header.frame_id;
        ci1->header.stamp    = img_msg_RosTime[i].header.stamp;

        // publish the image
        image_pub_RosTime[i].publish(img_msg_RosTime[i], *ci1);
      }
#endif
      return true;
    }
    return false;
  }

  bool spin()
  {
    ros::Rate loop_rate(fps);
    while (node_.ok())
    {
      if (!grab_and_send_image())
        ROS_WARN("gmsl camera did not respond in time.");

      ros::spinOnce();
      loop_rate.sleep();
    }
    return true;
  }
};

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_miivii_gmsl");

  ros_miivii_gmsl::ros_miivii_gmsl_Node a;

  a.spin();
  return EXIT_SUCCESS;
}
