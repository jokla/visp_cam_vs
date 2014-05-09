#include "VSTask.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/CameraInfo.h>



#include <visp/vpImage.h>
#include <visp/vpDisplayX.h>

#ifndef __visp_cam_vs__
#define __visp_cam_vs__

class VScam
{
  ros::NodeHandle nh_;
  /**
  * Handler for the image transportation (see package "image_transport")
  */
  image_transport::ImageTransport it_;
  /**
  * Subscriber for the image transportation 
  */
  image_transport::Subscriber image_sub_;
  /**
  * Const pointer to the new image
  */
  sensor_msgs::ImageConstPtr imageIn_;
  /**
  * Publisher velocities
  */
  ros::Publisher vel_pub;
  /**
  * Publisher velocities
  */
  ros::Publisher feat_pub;
   /**
   * Last image ID  
   */
  unsigned int lastHeaderSeq_;
   /**
   * Visp image used for image processing and visualization
   */
  vpImage<unsigned char> I;
  /**
   * Hadler display for visualization
   */
  vpDisplayX display ;
  /**
   * Object to manage the Visual Servoing Task
   */
  VSTask Task;
  /**
   * StreamIsStarted is set to 1 when the images start to arrive
   */
  bool StreamIsStarted;
  /**
  * Velocity message
  */
  geometry_msgs::TwistStamped vel;
  /**
  * Actual Features message
  */
  geometry_msgs::PoseStamped act_features_msg;

  /**
  * Camera parameteres
  */
  vpCameraParameters infoCam;
  /**
  * Subscriber to sensor_msgs::CameraInfo
  */
  ros::Subscriber sub_cam_info;
  /**
  * Is equal to one if we received the information about the camera
  */
  bool Stream_info_camera;

  
public:
  /**
  * Constructor.
  */
  VScam();
  /**
  * Destructor
  */
  ~VScam();

  /**
  * Callback function called each time an image (msg) arrives from ROS ()
  */ 
  void imageCb(const sensor_msgs::ImageConstPtr& msg);
    /**
  * Callback function to manage the camera parameters
  */
  void CameraInfoCb(const sensor_msgs::CameraInfo& msg);
  /**
  * This function check if a new image is arrived. If yes we start the image processing and the visual servoing computation
  */   
  void process ();
  /**
  * This function call "SpidOnce" at a certain frequency that we specify.
  */     
  virtual void spin ();
  /**
  *  Within this function we call ros::spinOnce() (ROS  processes our callbacks) and the function process to check if a new image is ready
  */     
  virtual  void spinOnce ();
    
};

#endif 
