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
    Node ()
       // initialization list copy the arguments to the class attributes.
      {
       
       imageOut_.init(480,640);
       lastHeaderSeq_ = 1;

       image_transport::ImageTransport it_(nh_);

         //pub_ = it_.advertise("out", 1);
       sub_ = it_.subscribe("vrep/Vision_sensorIT", 1, &Node::subscriberCb,this);

        // create subscriber, publisher using nh_.
      }

     // always use shared pointer version of callbacks. sensor_msgs::ImageShPtr&
     //void subscriberCb (const boost::shared_ptr< sensor_msgs::ImagConstPtr >& image)
    void subscriberCb (const sensor_msgs::ImageConstPtr& image) 
       {
        std::cout << "Image Ricevuta"<<std::endl;

         //imageIn_ = image;
         std::cout << "Image Copiata"<<std::endl;

       }

    void process ()
    {
         if(&imageIn_ == NULL)
          return;

 //         std::cout << &imageIn_<<std::endl;
 //         // Use imageIn as your input image.
 //        std::cout << "enter process"<<std::endl;
 //         sensor_msgs::ImageConstPtr imageIn = imageIn_; // See note (1)
 //          try
 //          {
 //            //imageOut_ = cv_bridge::toCvCopy(imageIn, enc::BGR8); // OLD
 //         std::cout << "enter process 1"<<std::endl;
 //            //Convert ROS image to Visp Image
 //            //imageOut_  = visp_bridge::toVispImage(*imageIn);
           
 //          }
 //         catch (...)
 //          {
 //              std::cerr << "Error during the conversion from ROS image to Visp Image.";
 //              return;
 //          }

 // std::cout << "conversion ok"<<std::endl;


 //         // No new images since last time, do nothing.
 //           if (lastHeaderSeq_ == imageIn->header.seq)
 //           {
 //               std::cout << "no new images"<<std::endl;
 //            return;
 //           }
           


 //          lastHeaderSeq_=imageIn->header.seq;

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
    image_transport::Subscriber sub_;
    image_transport::Publisher pub_;
    ros::NodeHandle nh_;
   

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
    NodeWithGui() : Node (){

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
 //ros::NodeHandle nh;

 //image_transport::ImageTransport it(nh);

 Node* node = 0;
 // if (ros::param::get<bool>("~use_gui"))
  //node = new NodeWithGui ();

  //else
   node = new Node ();

  std::cout << "EXIT"<<std::endl;
  node->spin ();
  return 0;
}