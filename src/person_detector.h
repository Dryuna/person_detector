#ifndef PERSON_DETECTOR_H
#define PERSON_DETECTOR_H
#include <ros/ros.h>                                    //general ROS-functionalities
#include <cob_people_detection_msgs/DetectionArray.h>   //Message type for the cob_people_detection_topic
#include <queue>                                       //used to store the detections
#include <vector>
#include <tf/transform_listener.h>                      //currently unused
#include <tf/transform_broadcaster.h>                   //used to broadcast detections
#include <visualization_msgs/Marker.h>                  //display markers on rviz
#include <person_detector/DetectionObjectArray.h>       //our detections
#include <person_detector/DetectionObject.h>            //used for a single detection

class person_detector_class
{
private:
  //ros-stuff
  ros::NodeHandle n_;
  ros::Subscriber sub_face_recognition_;
  ros::Publisher pub_all_recognitions_;
  //transformations
  tf::TransformListener tf_listener_;
  tf::StampedTransform transform_li_;
  tf::TransformBroadcaster tf_local_human_broadcaster_;
  tf::Transform transform_br_;
  tf::TransformBroadcaster tf_map_human_broadcaster_;
  tf::Transform transform_br_map_;

  //markers for rviz
  ros::Publisher human_marker_pub_;
  visualization_msgs::Marker points;
  ros::Publisher human_marker_text_pub_;
  visualization_msgs::Marker face_text;


  //callbacks
  void faceRecognitionCallback_(const cob_people_detection_msgs::DetectionArray received_detections);

  //variables
  std::queue<cob_people_detection_msgs::DetectionArray> detection_temp_storage_;
  bool detection_temp_storage_in_use_;
  person_detector::DetectionObjectArray all_detections_array_;
  bool detection_array_in_use_;

  //functions
  int preprocessDetections_();
  int classifyDetections_( cob_people_detection_msgs::DetectionArray detection_array );
  int addNewDetection(cob_people_detection_msgs::Detection new_detection);
  int updateDetection(cob_people_detection_msgs::Detection new_detection, unsigned int det_id);
  int findDistanceWinner_(std::vector< std::vector <double> > &distances, std::vector<unsigned int> &win_id, std::vector<double> &win_dist, unsigned int detection_array_size);
  int clearDoubleResults_(std::vector< std::vector <double> > &distances, std::vector<unsigned int> &win_id, std::vector<double> &win_dist, unsigned int detection_array_size);

public:
  person_detector_class();
  int run();
};

#endif // PERSON_DETECTOR_H
