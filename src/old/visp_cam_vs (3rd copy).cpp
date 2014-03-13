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



class Node
{
  public:
    Node (ros::NodeHandle& nh, ros::NodeHandle& nhPrivate) :
      nh_(*nh),
      nhPrivate_(*nhPrivate)
             // initialization list copy the arguments to the class attributes.
      {
        //nh_ = nh;
        //nhPrivate_=nhPrivate;
        //it_ = *it; 
        it_(nh_);

        imageOut_.init(480,640);
        lastHeaderSeq_ = 1;

         //pub_ = it_.advertise("out", 1);
         sub_ = it_.subscribe("vrep/Vision_sensorIT", 1, &Node::subscriberCb,this);

        // create subscriber, publisher using nh_.
      }

     // always use shared pointer version of callbacks. sensor_msgs::ImageShPtr&
     //void subscriberCb (const boost::shared_ptr< sensor_msgs::ImagConstPtr >& image)
    void subscriberCb (const sensor_msgs::ImageConstPtr& image) 
       {
         imageIn_ = image;
       }

    void process ()
    {
        // if(&imageIn_ == NULL)
         // return;
         // Use imageIn as your input image.
         sensor_msgs::ImageConstPtr imageIn = imageIn_; // See note (1)
           

          


         // No new images since last time, do nothing.
           if (lastHeaderSeq_ == imageIn->header.seq)
           return;

         else

         {

             //Convert ROS image to Visp Image
              imageOut_  = visp_bridge::toVispImage(*imageIn);

             lastHeaderSeq_=imageIn->header.seq;
         }


         

         // use imageOut_.image and images which are *class attributes*
         // to realize the image processing, so it will allocate memory only once.
         // You should have *no* instance of big objects as local variables here.
         // Use directly imageOut_.image as the cv::Mat destination object  of your last
         // OpenCV function call.

          //publish imageOut_.toMsg ();
          //*** pub_.publish(imageOut_->toImageMsg()); OLD
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

    virtual void spinOnce ()
      {
         process ();
         spinOnce ();
      }


  protected:
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_;
    image_transport::Publisher pub_;
    ros::NodeHandle nh_;
    ros::NodeHandle nhPrivate_;

    unsigned int lastHeaderSeq_;

     //CvImageConstPtr imageIn_;
     sensor_msgs::ImageConstPtr imageIn_ ;
     //*** cv_bridge::CvImagePtr imageOut_; OLD

     //Image in visp format
      vpImage<unsigned char> imageOut_;

 };

 class NodeWithGui : public Node
 {
  public:
    NodeWithGui(ros::NodeHandle& nh, ros::NodeHandle& nhPrivate) : Node (nh,nhPrivate){

       display.init(imageOut_, 0, 0, "Camera view");
    }
    virtual void spinOnce ()
    {
       Node::spinOnce ();
       // and refresh GUI using imshow.
       // cv::imshow("Corners",imageOut_.image);
      vpDisplay::display(imageOut_);
      vpDisplay::flush(imageOut_);
    }

  protected:
    vpDisplayX display;
 };

int main(int argc, char **argv)
{
     std::cout << "NodeCreated"<<std::endl;

 ros::init(argc, argv, "visp_cam_vs");
 ros::NodeHandle nh;
 ros::NodeHandle nhPrivate ("~");
 

 Node* node = 0;
 // if (ros::param::get<bool>("~use_gui"))
  //node = new NodeWithGui (nh, nhPrivate);

  //else
   node = new Node (nh, nhPrivate);

     std::cout << "EXIT"<<std::endl;
 // node->spin ();
  return 0;
}
