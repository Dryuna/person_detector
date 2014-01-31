#include "person_detector.h"
#include <ros/ros.h>
#include <cob_people_detection_msgs/DetectionArray.h>
#include <std_msgs/String.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "person_detector");
  person_detector mainDetector;

  mainDetector.run();
  return 0;
}

void person_detector::faceRecognitionCallback_(const cob_people_detection_msgs::DetectionArray received_detections)
{
  detectionStorage_.push_back(received_detections);
  ROS_INFO("Our new detection has the following lable: %s",received_detections.detections.front().label.c_str());
  ROS_DEBUG("Added a detection to the detectionStorage");
}

person_detector::person_detector()
{
  //initialize ros-stuff
  subFaceRecognition_ = n_.subscribe("/cob_people_detection/face_recognizer/face_recognitions",10, &person_detector::faceRecognitionCallback_,this);

}

int person_detector::run()
{
  ROS_INFO("Our maximum size is %i",detectionStorage_.max_size());
  ROS_INFO("Our capacity is %i",detectionStorage_.capacity());
  ros::Rate r(0.2);
  while (ros::ok())
  {
      if (detectionStorage_.size() > 0)
        {
          ROS_INFO("Our storage holds currently %i datasets",detectionStorage_.size());
          for(std::vector<cob_people_detection_msgs::DetectionArray>::iterator it = detectionStorage_.begin(); it != detectionStorage_.end(); ++it) {
              if (it->detections.size() > 0)
              {
                  for (std::vector<cob_people_detection_msgs::Detection>::iterator i = it->detections.begin(); i != it->detections.end(); ++i)
                    {
                      ROS_INFO("Detection name is %s",i->label.c_str());
                    }
               }
          }

        }
    //do crazy stuff
      r.sleep();
    ros::spinOnce();
  }
}
