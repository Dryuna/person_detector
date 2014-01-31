#ifndef PERSON_DETECTOR_H
#define PERSON_DETECTOR_H
#include <ros/ros.h>                                    //general ROS-functionalities
#include <cob_people_detection_msgs/DetectionArray.h>   //Message type for the cob_people_detection_topic
#include <vector>                                       //used to store the detections

class person_detector
{
private:
  //ros-stuff
  ros::NodeHandle n_;
  ros::Subscriber subFaceRecognition_;

  //callbacks
  void faceRecognitionCallback_(const cob_people_detection_msgs::DetectionArray received_detections);

  //variables
  std::vector<cob_people_detection_msgs::DetectionArray> detectionStorage_;


public:
  person_detector();
  int run();
};

#endif // PERSON_DETECTOR_H
