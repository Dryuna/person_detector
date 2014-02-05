#include "person_detector.h"
#include <ros/ros.h>
#include <ros/time.h>
#include <cob_people_detection_msgs/DetectionArray.h>
#include <std_msgs/String.h>
#include <boost/lexical_cast.hpp>   //to cast the integer


int main(int argc, char** argv)
{
  ros::init(argc, argv, "person_detector");
  person_detector_class mainDetector;

  mainDetector.run();
  return 0;
}

void person_detector_class::faceRecognitionCallback_(const cob_people_detection_msgs::DetectionArray received_detections)
{
  ROS_INFO("Added a detection to the detectionStorage");
  cob_people_detection_msgs::DetectionArray temp_detections = received_detections;
//  ROS_INFO("ROS Time in sec: %f, PersonDetection Time in sec: %f",ros::Time::now().toSec(),temp_detections.header.stamp.toSec());

  if (!temp_detections.detections.empty())
  {
      for (unsigned int it = 0; it < temp_detections.detections.size(); it++)
      {
        ROS_INFO("Our new detection %i has the following lable: %s",it+1,temp_detections.detections[it].label.c_str());
        transform_br_.setOrigin(tf::Vector3(temp_detections.detections[it].pose.pose.position.x,
                                           temp_detections.detections[it].pose.pose.position.y,
                                           temp_detections.detections[it].pose.pose.position.z));

        transform_br_.setRotation(tf::Quaternion(temp_detections.detections[it].pose.pose.orientation.x,
                                                temp_detections.detections[it].pose.pose.orientation.y,
                                                temp_detections.detections[it].pose.pose.orientation.z));
        std::string pose_name = "/person_detector/human_local_pose_raw_" + boost::lexical_cast<std::string>((it+1));
        tf_local_human_broadcaster_.sendTransform(tf::StampedTransform(transform_br_,ros::Time::now(),"/camera_rgb_optical_frame",pose_name));
        //add tf to the object
        std_msgs::Header tempHeader;
        tempHeader.frame_id =  pose_name;
        temp_detections.detections[it].pose.header.frame_id = tempHeader.frame_id;
        geometry_msgs::Point p;
        p.x = temp_detections.detections[it].pose.pose.position.x;
        p.y = temp_detections.detections[it].pose.pose.position.y;
        p.z = temp_detections.detections[it].pose.pose.position.z;
        points.header.stamp = ros::Time::now();
        points.id = it;
        points.points.push_back(p);
        human_marker_pub_.publish(points);
        points.points.clear();

        face_text.header.stamp = ros::Time::now();
        face_text.id = it;
        face_text.text = temp_detections.detections[it].label;
        face_text.pose.position.x = temp_detections.detections[it].pose.pose.position.x;
        face_text.pose.position.y = temp_detections.detections[it].pose.pose.position.y;
        face_text.pose.position.z = temp_detections.detections[it].pose.pose.position.z+0.3;
        human_marker_text_pub_.publish(face_text);
      }
      detection_temp_storage_.push(temp_detections);
  }
}

int person_detector_class::preprocessDetections_()
{
  cob_people_detection_msgs::DetectionArray temporary_detection_array;
//  ros::Rate rp(100);
//  while(ros::ok())
//  {
    if (!detection_temp_storage_.empty())
    {
      //Take the first element to process it
      temporary_detection_array = detection_temp_storage_.front();
      //Skip everything, if we didn't detect anything new
      if (temporary_detection_array.detections.size() == 0)
      {
        all_detections_array_.header.stamp = ros::Time::now();
        detection_temp_storage_.pop();
        return 0;
      }
      //Transform all detections into map coordinates
      for (unsigned int it = 0; it < temporary_detection_array.detections.size(); it++)
      {
        tf_listener_.waitForTransform("/map", temporary_detection_array.detections[it].pose.header.frame_id ,temporary_detection_array.header.stamp,ros::Duration(0.25));
        try
        {
          tf_listener_.lookupTransform("/map",temporary_detection_array.detections[it].pose.header.frame_id,temporary_detection_array.header.stamp,transform_li_);
        }
        catch (tf::TransformException ex)
        {
          ROS_ERROR("%s",ex.what());
          ROS_ERROR("Not using this detection.");
          detection_temp_storage_.pop();
          return 1;
        }
        ROS_INFO("The transform says for x: %f for y: %f and for z: %f",transform_li_.getOrigin().x(),transform_li_.getOrigin().y(),transform_li_.getOrigin().z());
        temporary_detection_array.detections[it].pose.header.frame_id = "/map";
        temporary_detection_array.detections[it].pose.pose.position.x = transform_li_.getOrigin().x();
        temporary_detection_array.detections[it].pose.pose.position.y = transform_li_.getOrigin().y();
        temporary_detection_array.detections[it].pose.pose.position.z = transform_li_.getOrigin().z();
        temporary_detection_array.detections[it].pose.pose.orientation.x = transform_li_.getRotation().x();
        temporary_detection_array.detections[it].pose.pose.orientation.y = transform_li_.getRotation().y();
        temporary_detection_array.detections[it].pose.pose.orientation.z = transform_li_.getRotation().z();
        temporary_detection_array.detections[it].pose.pose.orientation.w = transform_li_.getRotation().w();
        transform_br_map_.setOrigin(tf::Vector3(temporary_detection_array.detections[it].pose.pose.position.x,
                                           temporary_detection_array.detections[it].pose.pose.position.y,
                                           temporary_detection_array.detections[it].pose.pose.position.z));

        transform_br_map_.setRotation(tf::Quaternion(temporary_detection_array.detections[it].pose.pose.orientation.x,
                                                temporary_detection_array.detections[it].pose.pose.orientation.y,
                                                temporary_detection_array.detections[it].pose.pose.orientation.z));
        std::string pose_name = "/person_detector/human_pose_raw_" +  boost::lexical_cast<std::string>((it+1));
        tf_local_human_broadcaster_.sendTransform(tf::StampedTransform(transform_br_map_,ros::Time::now(),"/map",pose_name));
      }
      //push object to the classification
      classifyDetections_( temporary_detection_array);
      detection_array_in_use_ = false;
      detection_temp_storage_.pop();
    }
    //publish the new array
    pub_all_recognitions_.publish(all_detections_array_);

    return 0;



//  rp.sleep();
//  }
}

int person_detector_class::classifyDetections_( cob_people_detection_msgs::DetectionArray detection_array )
{
  // push directly, if it is empty
  if (all_detections_array_.detections.empty())
  {
    //just initialize the result and push it to the detection_array
    for (unsigned int it = 0; it < detection_array.detections.size(); it++)
    {
      addNewDetection(detection_array.detections[it]);
      ROS_INFO("First detection - added a new one");
    }
    return 0;
  }
  ROS_INFO("Started distance calculation");
  //try to find a corresponging match within a distance of x cm
  double max_dist_m = 0.50;
  std::vector< std::vector <double> > distances (detection_array.detections.size(), std::vector<double>(all_detections_array_.detections.size()));
  //populate vector with
  for (unsigned int in = 0; in < detection_array.detections.size(); in++)
  {    
    //calculate the distance to all known detections
    for (unsigned int ia = 0; ia < all_detections_array_.detections.size(); ia++)
    {
        double xdiff = detection_array.detections[in].pose.pose.position.x - all_detections_array_.detections[ia].latest_pose_map.pose.position.x;
        double ydiff = detection_array.detections[in].pose.pose.position.y - all_detections_array_.detections[ia].latest_pose_map.pose.position.y;
        double zdiff = detection_array.detections[in].pose.pose.position.z - all_detections_array_.detections[ia].latest_pose_map.pose.position.z;
        //calculate
        distances[in][ia] = sqrt((xdiff*xdiff)+(ydiff*ydiff)+(zdiff*zdiff));
    }
  }

  //////////////////
  //find the winner
  //////////////////
  ROS_INFO("Searching for the winner");
  std::vector<unsigned int> win_id (detection_array.detections.size());
  std::vector<double> win_dist (detection_array.detections.size());
  //populate vector with incredible high values
  for (unsigned int in = 0; in < detection_array.detections.size(); in++)
  {
      win_dist[in] = 1000000;
  }
  findDistanceWinner_(distances,win_id,win_dist,detection_array.detections.size());
  //check for double results
  //clearDoubleResults

  for (unsigned int in = 0; in < detection_array.detections.size(); in++)
  {
    // either update a result or make a new result
    if (win_dist[in] < max_dist_m )
    {
      updateDetection(detection_array.detections[in], win_id[in]);
    } else
    {
      addNewDetection(detection_array.detections[in]);
    }
  }
  return 0;
}

int person_detector_class::addNewDetection(cob_people_detection_msgs::Detection new_detection)
{
  ROS_INFO("Addind a new detection");
  person_detector::DetectionObject push_object;
  person_detector::NameLabel push_name;
  push_object.header.stamp = push_object.latest_pose_map.header.stamp = ros::Time::now();
  push_object.latest_pose_map.header.frame_id = "/map";
  push_object.total_detections = 1;
  push_object.latest_pose_map.pose.position.x = new_detection.pose.pose.position.x;
  push_object.latest_pose_map.pose.position.y = new_detection.pose.pose.position.y;
  push_object.latest_pose_map.pose.position.z = new_detection.pose.pose.position.z;
  push_object.confirmation.label = "";
  push_object.confirmation.running = false;
  push_object.confirmation.suceeded = false;
  push_object.confirmation.tried = false;
  if (new_detection.label != "UnknownHead")
  {
      push_name.label = new_detection.label;
      push_name.quantity = 1;
      push_object.recognitions.total_assigned = 1;
  } else
  {
    push_object.recognitions.total_assigned = 0;
  }
  push_object.recognitions.name_array.push_back(push_name);
  all_detections_array_.detections.push_back(push_object);
  all_detections_array_.header.stamp = ros::Time::now();
  all_detections_array_.header.seq++;
}

int person_detector_class::updateDetection(cob_people_detection_msgs::Detection new_detection, unsigned int det_id)
{
  ROS_INFO("Updating a detection");
  //update general stuff
  all_detections_array_.header.stamp = ros::Time::now();
  all_detections_array_.header.seq++;

  //update our specific result
  all_detections_array_.detections[det_id].total_detections++;
  all_detections_array_.detections[det_id].latest_pose_map.pose.position.x = new_detection.pose.pose.position.x;
  all_detections_array_.detections[det_id].latest_pose_map.pose.position.y = new_detection.pose.pose.position.y;
  all_detections_array_.detections[det_id].latest_pose_map.pose.position.z = new_detection.pose.pose.position.z;
  all_detections_array_.detections[det_id].latest_pose_map.header.stamp = ros::Time::now();

  //update or create a new nametag
  bool found = false;
  std::string it_label;
  if (new_detection.label != "UnknownHead")
  {
    for (unsigned int it = 0; it < all_detections_array_.detections[det_id].recognitions.name_array.size(); it++)
    {
      it_label = all_detections_array_.detections[det_id].recognitions.name_array[it].label;
      if (new_detection.label == all_detections_array_.detections[det_id].recognitions.name_array[it].label)
      {
          all_detections_array_.detections[det_id].recognitions.name_array[it].quantity++;
          all_detections_array_.detections[det_id].recognitions.total_assigned++;
          found = true;
      }
    }
    if (!found)
      {
        person_detector::NameLabel push_name;
        push_name.label = new_detection.label;
        push_name.quantity = 1;
        all_detections_array_.detections[det_id].recognitions.name_array.push_back(push_name);
        all_detections_array_.detections[det_id].recognitions.total_assigned++;
      }
  }
  return 0;
}


int person_detector_class::findDistanceWinner_(std::vector< std::vector <double> > &distances, std::vector<unsigned int> &win_id, std::vector<double> &win_dist, unsigned int detection_array_size)
{
  for (unsigned int in = 0; in < detection_array_size; in++)
  {
    for (unsigned int ia = 0; ia < all_detections_array_.detections.size(); ia++)
    {
        if (distances[in][ia] < win_dist[in] )
          {
            win_dist[in] = distances[in][ia];
            win_id[in] = ia;
          }
    }
  }
  return 0;
}

int person_detector_class::clearDoubleResults_(std::vector< std::vector <double> > &distances, std::vector<unsigned int> &win_id, std::vector<double> &win_dist, unsigned int detection_array_size)
{
  //if we had just one new detection, we are done - puh
  if (detection_array_size == 1)
  {
    return 0;
  }
  //if we have several new detections, we have to check, because we don't want to match results together
  std::vector<unsigned int> same_ids;
  for (unsigned int in = 0; in < detection_array_size; in++)
  {
      bool in_clear = false;
      while (!in_clear)
      {
        same_ids.push_back(win_id[in]);
        for (unsigned int in2 = 1; in < detection_array_size; in2++)
          {
            if (win_id[in] == win_id[in2])
              {
                same_ids.push_back(in2);
              }
          }
        if (same_ids.size() < 2) {
            in_clear = true;
          } else
          {
            for (unsigned int ir = 0; ir < same_ids.size(); ir++)
               {

              }

          }
      }


  }

}

person_detector_class::person_detector_class()
{
  //initialize ros-stuff
  sub_face_recognition_ = n_.subscribe("/cob_people_detection/detection_tracker/face_position_array",10, &person_detector_class::faceRecognitionCallback_,this);
  pub_all_recognitions_ = n_.advertise<person_detector::DetectionObjectArray>("/person_detector/all_recognitions",10);
  //initialize markers
  human_marker_pub_ = n_.advertise<visualization_msgs::Marker>("/person_detector/face_marker_raw",10);
  human_marker_text_pub_ = n_.advertise<visualization_msgs::Marker>("/person_detector/face_marker_text_raw",10);

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
 // boost::thread process_thread_object (&person_detector_class::processDetections, this);
  ros::Rate r(100);
  ros::Time start;
  ros::Time end;
  ros::Duration difference;
  while (ros::ok())
  {
    start = ros::Time::now();
    preprocessDetections_();
    end = ros::Time::now();
    difference = end-start;
    if (detection_temp_storage_.size() > 10) ROS_WARN("Our temporary storage is too big. It holds %i objects. Last circle took %f seconds.",detection_temp_storage_.size(),difference.toSec());
    //do crazy stuff
    r.sleep();
    ros::spinOnce();
  }
}
