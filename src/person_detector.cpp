#include "person_detector.h"
#include <ros/ros.h>
#include <ros/time.h>
#include <cob_people_detection_msgs/DetectionArray.h>
#include <std_msgs/String.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "person_detector");
  person_detector_class mainDetector;

  mainDetector.run();
  return 0;
}

void person_detector_class::faceRecognitionCallback_(const cob_people_detection_msgs::DetectionArray received_detections)
{
  detection_storage_.push(received_detections);
  ROS_INFO("Added a detection to the detectionStorage");
//  ROS_INFO("ROS Time in sec: %f, PersonDetection Time in sec: %f",ros::Time::now().toSec(),received_detections.header.stamp.toSec());
//  tfListener_.waitForTransform("/camera_rgb_optical_frame","/map",received_detections.header.stamp,ros::Duration(60.0));
//  try
//  {
//    tfListener_.lookupTransform("/camera_rgb_optical_frame","/map",received_detections.header.stamp,transform_);
//  }
//  catch (tf::TransformException ex)
//  {
//    ROS_ERROR("%s",ex.what());
//  }

//  ROS_INFO("The transform says for x: %f and for y: %f",transform_.getOrigin().x(),transform_.getOrigin().y());
  if (!received_detections.detections.empty())
  {
      for (unsigned int it = 0; it < received_detections.detections.size(); it++)
      {
      ROS_INFO("Our new detection %i has the following lable: %s",it+1,received_detections.detections[it].label.c_str());
      transform_br_.setOrigin(tf::Vector3(received_detections.detections[it].pose.pose.position.x,
                                         received_detections.detections[it].pose.pose.position.y,
                                         received_detections.detections[it].pose.pose.position.z));

      transform_br_.setRotation(tf::Quaternion(received_detections.detections[it].pose.pose.orientation.x,
                                              received_detections.detections[it].pose.pose.orientation.y,
                                              received_detections.detections[it].pose.pose.orientation.z));
      std::string pose_name = "/person_detector/human_pose_raw_" + (it+1);
      tf_human_broadcaster_.sendTransform(tf::StampedTransform(transform_br_,ros::Time::now(),"/camera_rgb_optical_frame",pose_name));

      geometry_msgs::Point p;
      p.x = received_detections.detections[it].pose.pose.position.x;
      p.y = received_detections.detections[it].pose.pose.position.y;
      p.z = received_detections.detections[it].pose.pose.position.z;
      points.header.stamp = ros::Time::now();
      points.id = it;
      points.points.push_back(p);
      human_marker_pub_.publish(points);
      points.points.clear();

      face_text.header.stamp = ros::Time::now();
      face_text.id = it;
      face_text.text = received_detections.detections[it].label;
      face_text.pose.position.x = received_detections.detections[it].pose.pose.position.x;
      face_text.pose.position.y = received_detections.detections[it].pose.pose.position.y;
      face_text.pose.position.z = received_detections.detections[it].pose.pose.position.z+0.3;
      human_marker_text_pub_.publish(face_text);
      }
  }
}

int person_detector_class::processDetections()
{
  cob_people_detection_msgs::DetectionArray temporary_detection_array;
  ros::Rate(10);
  while(ros::ok())
  {
    if (!detection_storage_.empty())
    {
      temporary_detection_array = detection_storage_.front();
      detection_storage_.pop();
      detection_array_in_use_ = true;
      detection_array_in_use_ = false;
    }
  }
}

person_detector_class::person_detector_class()
{
  //initialize ros-stuff
  sub_face_recognition_ = n_.subscribe("/cob_people_detection/face_recognizer/face_recognitions",10, &person_detector_class::faceRecognitionCallback_,this);
  //initialize markers
  human_marker_pub_ = n_.advertise<visualization_msgs::Marker>("robot_control/face_marker_raw",10);
  human_marker_text_pub_ = n_.advertise<visualization_msgs::Marker>("robot_control/face_marker_text_raw",10);

  points.header.frame_id = "/camera_rgb_optical_frame";
  points.ns = "person_detector/face_marker";
  points.id = 0;
  points.lifetime = ros::Duration(10);
  points.action = visualization_msgs::Marker::ADD;
  points.type = visualization_msgs::Marker::POINTS;
  points.scale.x = 0.2;
  points.scale.y = 0.2;
  points.scale.z = 0.2;
  points.color.g = 1.0f;
  points.color.a = 1.0;

  face_text.header.frame_id = "/camera_rgb_optical_frame";
  face_text.ns = "person_detector/face_marker_text";
  face_text.id = 0;
  face_text.lifetime = ros::Duration(10);
  face_text.action = visualization_msgs::Marker::ADD;
  face_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  face_text.scale.z = 0.2;
  face_text.color.r = 1.0f;
  face_text.color.a = 1.0;

  //initialize array
  detection_array_in_use_ = false;
}

int person_detector_class::run()
{
  // An extra thread to do the processing of incoming data
  boost::thread process_thread_object (&person_detector_class::processDetections, this);
  ros::Rate r(10);
  while (ros::ok())
  {
      if (detection_storage_.size() > 10) ROS_WARN("Our temporary storage is too big. It holds %i object",detection_storage_.size());
//      if (detectionStorage_.size() > 0)
//        {
//          ROS_INFO("Our storage holds currently %i datasets",detectionStorage_.size());
//          for(std::vector<cob_people_detection_msgs::DetectionArray>::iterator it = detectionStorage_.begin(); it != detectionStorage_.end(); ++it) {
//              if (it->detections.size() > 0)
//              {
//                  for (std::vector<cob_people_detection_msgs::Detection>::iterator i = it->detections.begin(); i != it->detections.end(); ++i)
//                    {
//                      ROS_INFO("Detection name is %s",i->label.c_str());
//                    }
//               }
//          }

//        }
    //do crazy stuff
      r.sleep();
    ros::spinOnce();
  }
}
