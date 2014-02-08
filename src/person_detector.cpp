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
  ROS_DEBUG("Added a detection to the detectionStorage");
  cob_people_detection_msgs::DetectionArray temp_detections = received_detections;
//  ROS_DEBUG("ROS Time in sec: %f, PersonDetection Time in sec: %f",ros::Time::now().toSec(),temp_detections.header.stamp.toSec());

  if (!temp_detections.detections.empty())
  {
      for (unsigned int it = 0; it < temp_detections.detections.size(); it++)
      {
        ROS_DEBUG("Our new detection %i has the following lable: %s",it+1,temp_detections.detections[it].label.c_str());
        transform_br_.setOrigin(tf::Vector3(temp_detections.detections[it].pose.pose.position.x,
                                           temp_detections.detections[it].pose.pose.position.y,
                                           temp_detections.detections[it].pose.pose.position.z));

        transform_br_.setRotation(tf::Quaternion(temp_detections.detections[it].pose.pose.orientation.x,
                                                temp_detections.detections[it].pose.pose.orientation.y,
                                                temp_detections.detections[it].pose.pose.orientation.z));
        std::string pose_name = "/person_detector/human_local_pose_raw_" + boost::lexical_cast<std::string>((it+1));
        tf_human_local_broadcaster_.sendTransform(tf::StampedTransform(transform_br_,ros::Time::now(),"/camera_rgb_optical_frame",pose_name));
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
        human_marker_raw_pub_.publish(points);
        points.points.clear();

        face_text.header.stamp = ros::Time::now();
        face_text.id = it;
        face_text.text = temp_detections.detections[it].label;
        face_text.pose.position.x = temp_detections.detections[it].pose.pose.position.x;
        face_text.pose.position.y = temp_detections.detections[it].pose.pose.position.y;
        face_text.pose.position.z = temp_detections.detections[it].pose.pose.position.z+0.3;
        human_marker_raw_text_pub_.publish(face_text);
      }
      detection_temp_storage_.push(temp_detections);
  }
}

void person_detector_class::allRecognitionsCallback_(const person_detector::DetectionObjectArray all_detections)
{
  //check if we don't have any recognitions
  if (all_detections.detections.empty())
  {
    return;
  }
  geometry_msgs::Point p;
  std::string name;
  for (unsigned int it = 0; it < all_detections.detections.size(); it++)
  {
      //declare positions and stamp
      p.x = all_detections.detections[it].latest_pose_map.pose.position.x;
      p.y = all_detections.detections[it].latest_pose_map.pose.position.y;
      p.z = all_detections.detections[it].latest_pose_map.pose.position.z;
      heads_.header.stamp = ros::Time::now();
      heads_.id = all_detections.detections[it].header.seq;
      heads_.points.push_back(p);
      //change colour?

      pub_human_marker_.publish(heads_);
      heads_.points.clear();
      //make text
      //find dominating name
      if (all_detections.detections[it].recognitions.name_array.empty())
        {
          int sec = (ros::Time::now().toSec() - all_detections.detections[it].latest_pose_map.header.stamp.toSec());
          name = "Unknown | " +  boost::lexical_cast<std::string>((sec)) + "s";
        } else {
          int hits = 0;
          for (unsigned int in = 0; in < all_detections.detections[it].recognitions.name_array.size(); in++)
            {
              if (all_detections.detections[it].recognitions.name_array[in].quantity > hits)
                {
                  name = all_detections.detections[it].recognitions.name_array[in].label + " ";
                  //ROS_DEBUG("Calculating percentage with quantity %i and total detections %",all_detections.detections[it].recognitions.name_array[in].quantity,all_detections.detections[it].total_detections);
                  double percentage = ((all_detections.detections[it].recognitions.name_array[in].quantity) * 100 / all_detections.detections[it].total_detections);
                  name += boost::lexical_cast<std::string>(percentage);
                  hits = all_detections.detections[it].recognitions.name_array[in].quantity;
                }
            }
          int cast = all_detections.detections[it].recognitions.total_assigned;
          //ROS_DEBUG("The major results name and percentage is %s",name.c_str());
          name += "% of " + boost::lexical_cast<std::string>(cast) + " | ";
          int sec = (ros::Time::now().toSec() - all_detections.detections[it].latest_pose_map.header.stamp.toSec());
          name += boost::lexical_cast<std::string>(sec) + "s";
        }
      heads_text_.header.stamp = ros::Time::now();
      heads_text_.id = all_detections.detections[it].header.seq;
      heads_text_.text = name;
      heads_text_.pose.position.x = all_detections.detections[it].latest_pose_map.pose.position.x;
      heads_text_.pose.position.y = all_detections.detections[it].latest_pose_map.pose.position.y;
      heads_text_.pose.position.z = all_detections.detections[it].latest_pose_map.pose.position.z+0.3;
      pub_human_marker_text_.publish(heads_text_);

      //broadcast the results on tf
      transform_br_map_.setOrigin(tf::Vector3(all_detections.detections[it].latest_pose_map.pose.position.x,
                                         all_detections.detections[it].latest_pose_map.pose.position.y,
                                         all_detections.detections[it].latest_pose_map.pose.position.z));

      transform_br_map_.setRotation(tf::Quaternion(all_detections.detections[it].latest_pose_map.pose.orientation.x,
                                              all_detections.detections[it].latest_pose_map.pose.orientation.y,
                                              all_detections.detections[it].latest_pose_map.pose.orientation.z));
      std::string pose_name = "/person_detector/human_pose_" + boost::lexical_cast<std::string>((all_detections.detections[it].header.seq));
      tf_map_human_broadcaster_.sendTransform(tf::StampedTransform(transform_br_map_,ros::Time::now(),"/map",pose_name));
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
        ROS_DEBUG("The transform says for x: %f for y: %f and for z: %f",transform_li_.getOrigin().x(),transform_li_.getOrigin().y(),transform_li_.getOrigin().z());
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
        tf_human_local_broadcaster_.sendTransform(tf::StampedTransform(transform_br_map_,ros::Time::now(),"/map",pose_name));
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
      ROS_DEBUG("First detection - added a new one");
    }
    return 0;
  }
  ROS_DEBUG("Started distance calculation");
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
  ROS_DEBUG("Searching for the winner");
  std::vector<unsigned int> win_id (detection_array.detections.size());
  std::vector<double> win_dist (detection_array.detections.size());
  //populate vector with incredible high values
  for (unsigned int in = 0; in < detection_array.detections.size(); in++)
  {
      win_dist[in] = 1000000;
  }
  findDistanceWinner_(distances,win_id,win_dist,detection_array.detections.size());
  //check for double results
  clearDoubleResults_(distances,win_id,win_dist,detection_array.detections.size());

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
  ROS_DEBUG("Addind a new detection");
  person_detector::DetectionObject push_object;
  person_detector::NameLabel push_name;
  push_object.header.stamp = push_object.latest_pose_map.header.stamp = ros::Time::now();
  push_object.header.seq = detection_id;
  detection_id++;
  push_object.latest_pose_map.header.frame_id = "/map";
  push_object.total_detections = 1;
  push_object.latest_pose_map.pose.position.x = new_detection.pose.pose.position.x;
  push_object.latest_pose_map.pose.position.y = new_detection.pose.pose.position.y;
  push_object.latest_pose_map.pose.position.z = new_detection.pose.pose.position.z;
  push_object.confirmation.label = "";
  push_object.confirmation.running = false;
  push_object.confirmation.suceeded = false;
  push_object.confirmation.tried = false;
  if (new_detection.label != "UnknownHead" && new_detection.label != "Unknown")
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
  substractHit(new_detection.label,(all_detections_array_.detections.size()-1));
  all_detections_array_.header.seq++;
}

int person_detector_class::updateDetection(cob_people_detection_msgs::Detection new_detection, unsigned int det_id)
{
  ROS_DEBUG("Updating a detection");
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
  if (new_detection.label != "UnknownHead" && new_detection.label != "Unknown")
  {
    for (unsigned int it = 0; it < all_detections_array_.detections[det_id].recognitions.name_array.size(); it++)
    {
      it_label = all_detections_array_.detections[det_id].recognitions.name_array[it].label;
      if (new_detection.label == all_detections_array_.detections[det_id].recognitions.name_array[it].label)
      {
          all_detections_array_.detections[det_id].recognitions.name_array[it].quantity++;
          all_detections_array_.detections[det_id].recognitions.total_assigned++;
          found = true;
          ROS_DEBUG("Found a hit and now substracting of the rest");
          substractHit(new_detection.label, det_id);
      }
    }
    if (!found)
      {
        person_detector::NameLabel push_name;
        push_name.label = new_detection.label;
        push_name.quantity = 1;
        all_detections_array_.detections[det_id].recognitions.name_array.push_back(push_name);
        all_detections_array_.detections[det_id].recognitions.total_assigned++;
        substractHit(new_detection.label, det_id);
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
  std::vector<bool> old_detect_avail (all_detections_array_.detections.size());
  std::vector<bool> new_recogn_avail (win_id.size());
  //populate with availability
  for (unsigned int it = 0; it < all_detections_array_.detections.size(); it++)
  {
    old_detect_avail [it] = true;
  }
  for (unsigned int it = 0; it < win_id.size(); it++)
  {
    new_recogn_avail[it] = true;
  }

  while (ros::ok())
  {
    //we're searching for the shortest distance first
    double closest = 100000;
    unsigned int clos_id = 0;
    for (unsigned int in = 0; in < win_dist.size(); in++)
    {
      if (win_dist[in] < closest && old_detect_avail[win_id[in]] )
      {
        clos_id = in;
        closest = win_dist[in];
      }
    }
    //make the win_id unattractive for all other new detections
    for (unsigned int it = 0; it < win_id.size(); it++)
    {
      if (it != clos_id)
      {
          distances[it][win_id[clos_id]] = 9999;
      }
    }
    //set results as taken
    old_detect_avail[win_id[clos_id]] = false;
    new_recogn_avail[clos_id] = false;
    //calculate new winners, now that we could exclude one
    findDistanceWinner_(distances,win_id,win_dist,win_id.size());
    //check, if we assigned all old detections
    bool more_work = false;
    for (unsigned int iw = 0; iw < old_detect_avail.size(); iw++)
    {
        if (old_detect_avail[iw] == true)
          {
            more_work = true;
          }
    }
    if (!more_work)
      {
        return 0;
      }
    //check if we assigned all new recognitions
    more_work = false;
    for (unsigned int in = 0; in < win_id.size(); in++)
    {
      if (new_recogn_avail[in] == true)
        {
          more_work = true;
        }
    }
    if (!more_work)
      {
        ROS_DEBUG("Finishing, because we assigned all new detections");
        return 0;
      }
  }
}



int person_detector_class::substractHit(std::string label, unsigned int leave_id)
{
  for (unsigned int it = 0; it < all_detections_array_.detections.size(); it++)
  {
    //we have to leave the one out, we just found
    if (it != leave_id)
      {
        for (unsigned int id = 0; id < all_detections_array_.detections[it].recognitions.name_array.size(); id++)
        {
          if (all_detections_array_.detections[it].recognitions.name_array[id].label == label)
          {
            if (all_detections_array_.detections[it].recognitions.name_array[id].quantity > 1)
              {
                all_detections_array_.detections[it].recognitions.name_array[id].quantity--;
                all_detections_array_.detections[it].recognitions.total_assigned--;
                all_detections_array_.detections[it].total_detections--;
              }
            else
              {
                all_detections_array_.detections[it].recognitions.name_array.erase(all_detections_array_.detections[it].recognitions.name_array.begin()+id );
                all_detections_array_.detections[it].recognitions.total_assigned--;
                all_detections_array_.detections[it].total_detections--;
              }
          }
        }
      }
  }
  return 0;
}

int person_detector_class::cleanDetectionArray(ros::Duration oldness)
{
  if (all_detections_array_.detections.empty())
  {
    return 0;
  }
  ros::Time present = ros::Time::now();
  ros::Time time_stamp;
  for (unsigned int it = 0; it < all_detections_array_.detections.size(); it++)
  {
      time_stamp = all_detections_array_.detections[it].latest_pose_map.header.stamp;
      if(all_detections_array_.detections[it].recognitions.name_array.empty() && ((present - time_stamp) > oldness))
        {
          all_detections_array_.detections.erase(all_detections_array_.detections.begin()+it);
        }
  }
}

person_detector_class::person_detector_class()
{
  //initialize ros-stuff
  sub_face_recognition_ = n_.subscribe("/cob_people_detection/detection_tracker/face_position_array",10, &person_detector_class::faceRecognitionCallback_,this);
  pub_all_recognitions_ = n_.advertise<person_detector::DetectionObjectArray>("/person_detector/all_recognitions",10);
  sub_all_recognitions_ = n_.subscribe("/person_detector/all_recognitions",10, &person_detector_class::allRecognitionsCallback_,this);
  //initialize markers
  human_marker_raw_pub_ = n_.advertise<visualization_msgs::Marker>("/person_detector/face_marker_raw",10);
  human_marker_raw_text_pub_ = n_.advertise<visualization_msgs::Marker>("/person_detector/face_marker_text_raw",10);
  pub_human_marker_ = n_.advertise<visualization_msgs::Marker>("/person_detector/human_marker",10);
  pub_human_marker_text_ = n_.advertise<visualization_msgs::Marker>("/person_detector/human_marker_text",10);

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

  heads_text_.header.frame_id = heads_.header.frame_id = "/map";
  heads_.ns = "person_detector/humans";
  heads_text_.lifetime = heads_.lifetime = ros::Duration(10);
  heads_text_.action = heads_.action = visualization_msgs::Marker::ADD;
  heads_.type = visualization_msgs::Marker::POINTS;
  heads_.scale.x = 0.3;
  heads_.scale.y = 0.3;
  heads_.scale.z = 0.3;
  heads_.color.r = 1.0;
  heads_.color.a = 1.0;

  heads_text_.ns = "person_detector/humans_text";
  heads_text_.scale.z = 0.2;
  heads_text_.color.r = 1.0;
  heads_text_.color.g = 1.0;
  heads_text_.color.a = 1.0;
  heads_text_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;



  //initialize array
  detection_array_in_use_ = false;
  detection_id = 0;

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
    cleanDetectionArray(ros::Duration(60));
    end = ros::Time::now();
    difference = end-start;
    if (detection_temp_storage_.size() > 10) ROS_WARN("Our temporary storage is too big. It holds %i objects. Last circle took %f seconds.",detection_temp_storage_.size(),difference.toSec());
    //do crazy stuff
    r.sleep();
    ros::spinOnce();
  }
}
