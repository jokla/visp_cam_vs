#include <iostream>

#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>


#include <visp_bridge/image.h>
#include <visp/vpImage.h>
#include <visp/vpDisplayX.h>

#include <visp/vpColor.h>
#include <visp/vpPolygon.h>
#include <visp/vpPixelMeterConversion.h>




class VScam
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  sensor_msgs::ImageConstPtr imageIn_;

  unsigned int lastHeaderSeq_;
  
  vpImage<unsigned char> I;
  vpDisplayX display ;

  
  
public:
  VScam()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("vrep/Vision_sensorIT", 1, &VScam::imageCb, this);
    lastHeaderSeq_ = 0;
    I.init(480,640);
    display.init(I, 0, 0, "Camera view");


    //image_pub_ = it_.advertise("/image_converter/output_video", 1);

  }

  ~VScam()
  {
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    
    imageIn_ = msg;

   // I = visp_bridge::toVispImage(*msg);  
    std::cout << msg->header.seq<<std::endl;

  }


 void process ()
    {

      if (imageIn_==0)
      {
         return;
      }
       // Use imageIn as your input image.
       sensor_msgs::ImageConstPtr imageIn = imageIn_; // See note (1)

       // No new images since last time, do nothing.
       if (lastHeaderSeq_ == imageIn->header.seq)
         return;

        I = visp_bridge::toVispImage(*imageIn);

          vpDisplay::display(I);
          vpDisplay::flush(I);

      
      lastHeaderSeq_ = imageIn->header.seq;
      std::cout << "Processing---->" << lastHeaderSeq_<<std::endl;
       // publish imageOut_.toMsg ();
    }

    virtual void spin ()
    {
      ros::Rate rate (30);
      while (ros::ok ())
      {
        
        spinOnce ();
        rate.sleep ();
       }
    }

  virtual  void spinOnce ()
    {
       process ();
       ros::spinOnce();
       //spinOnce ();
    }


};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "visp_cam_vs");
  VScam ic;
  //ros::spin();
  ic.spin ();
  return 0;
}