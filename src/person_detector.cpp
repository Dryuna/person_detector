#include "person_detector.h"
#include <ros/ros.h>                                            // general ROS-header
#include <ros/time.h>                                           // to use timestamps and durations
#include <cob_people_detection_msgs/DetectionArray.h>           // to process the detectionarrays provided by the cob-people-perception
#include <std_msgs/String.h>
#include <boost/lexical_cast.hpp>                               // to cast the integer
#include <math.h>                                               // to calculate difference
#include <sstream>                                              // to cast the chars
#include <std_msgs/ColorRGBA.h>                                 // to add colors to the lines

//! Creates an object of person_detector_class and runs the endless loop
int main(int argc, char** argv)
{
  ros::init(argc, argv, "person_detector");
  person_detector_class mainDetector;
  ROS_INFO("Finished initialization, now running in the loop");
  mainDetector.run();
  return 0;
}

/*! Incoming DetectionArrays are checked if they contain detections. Detections are published as a transformation based in the camera frame. Cubes and text are published on rviz and the detections are finally added to detection_temp_storage_ for further processing. */

void person_detector_class::faceRecognitionCallback_(const cob_people_detection_msgs::DetectionArray received_detections)
{
  cob_people_detection_msgs::DetectionArray temp_detections = received_detections;

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
      heads_raw.header.stamp = ros::Time::now();
      heads_raw.id = it;
      heads_raw.points.push_back(p);
      pub_human_marker_raw_.publish(heads_raw);
      heads_raw.points.clear();

      text_raw_.header.stamp = ros::Time::now();
      text_raw_.id = it;
      text_raw_.text = temp_detections.detections[it].label;
      text_raw_.pose.position.x = temp_detections.detections[it].pose.pose.position.x;
      text_raw_.pose.position.y = temp_detections.detections[it].pose.pose.position.y;
      text_raw_.pose.position.z = temp_detections.detections[it].pose.pose.position.z+0.3;
      pub_human_marker_raw_text_.publish(text_raw_);
    }
    ROS_DEBUG("Added a detection to the detectionStorage");
    detection_temp_storage_.push(temp_detections);
  }
}

/*! If the recognition array contains recognitions, they are displayed on rviz. A cube representing the face is displayed in map coordinates and a text displaying the ID of the detection, the most often appearing name, the percentage and the total number of matched recognitions are displayed. If a recognition is in the process of a confirmation or confirmed this is displayed as well. */

void person_detector_class::showAllRecognitions()
{
  //check if we don't have any recognitions
  if (all_detections_array_.detections.empty())
  {
    return;
  }
  geometry_msgs::Point p;
  std::string name;
  for (unsigned int it = 0; it < all_detections_array_.detections.size(); it++)
  {
    //declare positions and stamp
    p.x = all_detections_array_.detections[it].latest_pose_map.pose.position.x;
    p.y = all_detections_array_.detections[it].latest_pose_map.pose.position.y;
    p.z = all_detections_array_.detections[it].latest_pose_map.pose.position.z;
    heads_.header.stamp = ros::Time::now();
    heads_.id = all_detections_array_.detections[it].header.seq;

    //make text
    //find dominating name
    if (all_detections_array_.detections[it].recognitions.name_array.empty())
    {
      int sec = (ros::Time::now().toSec() - all_detections_array_.detections[it].latest_pose_map.header.stamp.toSec());
      name = "Unknown | " +  boost::lexical_cast<std::string>((sec)) + "s";
    }
    else
    {
      int hits = 0;
      for (unsigned int in = 0; in < all_detections_array_.detections[it].recognitions.name_array.size(); in++)
      {
        if (all_detections_array_.detections[it].recognitions.name_array[in].quantity > hits)
        {
          name = all_detections_array_.detections[it].recognitions.name_array[in].label + " ";
          ROS_DEBUG("Calculating percentage with quantity %i and total detections %",all_detections_array_.detections[it].recognitions.name_array[in].quantity,all_detections_array_.detections[it].total_detections);
          double percentage = ((all_detections_array_.detections[it].recognitions.name_array[in].quantity) * 100 / all_detections_array_.detections[it].total_detections);
          name += boost::lexical_cast<std::string>(percentage);
          hits = all_detections_array_.detections[it].recognitions.name_array[in].quantity;
        }
      }
      int cast = all_detections_array_.detections[it].recognitions.total_assigned;
      ROS_DEBUG("The major results name and percentage is %s",name.c_str());
      name += "% of " + boost::lexical_cast<std::string>(cast) + " | ";
      int sec = (ros::Time::now().toSec() - all_detections_array_.detections[it].latest_pose_map.header.stamp.toSec());
      name += boost::lexical_cast<std::string>(sec) + "s";
      //adding state, if we're
    }
    //changes in order of the state
    if (all_detections_array_.detections[it].confirmation.suceeded)
    {
        heads_.color.b = 0;
        heads_.color.g = 1.0;
        heads_.color.r = 0;
        name += "| confirmed";
    }
    else if (all_detections_array_.detections[it].confirmation.running)
    {
      heads_.color.g = 1.0;
      heads_.color.b = 0;
      heads_.color.r = 1.0;
      name += "| running";
    }
    else if (all_detections_array_.detections[it].confirmation.tried)
    {
      heads_.color.g = 1.0;
      heads_.color.b = 0;
      heads_.color.r = 1.0;
      name += "| tried";
    }
    else
    {
       heads_.color.b = 0;
       heads_.color.r = 1.0;
       heads_.color.g = 0;
    }
    //publishing this detection
    heads_.points.push_back(p);
    pub_human_marker_.publish(heads_);
    heads_.points.clear();

    heads_text_.header.stamp = ros::Time::now();
    heads_text_.id = all_detections_array_.detections[it].header.seq;
    heads_text_.text = name;
    heads_text_.pose.position.x = all_detections_array_.detections[it].latest_pose_map.pose.position.x;
    heads_text_.pose.position.y = all_detections_array_.detections[it].latest_pose_map.pose.position.y;
    heads_text_.pose.position.z = all_detections_array_.detections[it].latest_pose_map.pose.position.z+0.3;
    pub_human_marker_text_.publish(heads_text_);

    //broadcast the results on tf
    transform_br_map_.setOrigin(tf::Vector3(all_detections_array_.detections[it].latest_pose_map.pose.position.x,
                                       all_detections_array_.detections[it].latest_pose_map.pose.position.y,
                                       all_detections_array_.detections[it].latest_pose_map.pose.position.z));

    transform_br_map_.setRotation(tf::Quaternion(all_detections_array_.detections[it].latest_pose_map.pose.orientation.x,
                                            all_detections_array_.detections[it].latest_pose_map.pose.orientation.y,
                                            all_detections_array_.detections[it].latest_pose_map.pose.orientation.z));
    std::string pose_name = "/person_detector/human_pose_" + boost::lexical_cast<std::string>((all_detections_array_.detections[it].header.seq));
    tf_map_human_broadcaster_.sendTransform(tf::StampedTransform(transform_br_map_,ros::Time::now(),"/map",pose_name));
  }
}

/*! The coordinates are transformed from the human_pose_raw_X to the map frame. If it suceeds the detection will be classified and add or used for an update.
    \todo Using the human_pose_raw_X frame is probably a bad idea. It's better so switch to camera frame directly */

int person_detector_class::processDetections()
{
  cob_people_detection_msgs::DetectionArray temporary_detection_array;
    if (!detection_temp_storage_.empty())
    {
      temporary_detection_array = detection_temp_storage_.front();
      //Skip everything, if we didn't detect anything new
//      this case should be covered in the callback
//      if (temporary_detection_array.detections.size() == 0)
//      {
//        all_detections_array_.header.stamp = ros::Time::now();
//        detection_temp_storage_.pop();
//        return 0;
//      }
      //Transform all detections into map coordinates
      for (unsigned int it = 0; it < temporary_detection_array.detections.size(); it++)
      {
        //check, if the transform requested is older than the caches size
        ros::Duration t_diff (ros::Time::now() - temporary_detection_array.header.stamp);
        if (t_diff.toSec() > tf_cache_.toSec())
        {
          ROS_WARN("Not using this detection, because it's too old and no tf-data will be available");
          detection_temp_storage_.pop();
          return 1;
        }
        //wait a bit and try to do the lookup
        //tf_listener_.waitForTransform("/map", temporary_detection_array.detections[it].pose.header.frame_id ,temporary_detection_array.header.stamp,ros::Duration(0.25));

        //check if the detection is already available
        //ros::Time latest_tf;
        //std::string* error;
//        if (!tf_listener_.getLatestCommonTime(temporary_detection_array.detections[it].pose.header.frame_id,"/map",latest_tf,error))
//        {
//            ROS_ERROR("Can't lookup transform. There's probably something very wrong.");
//        }
//        if (temporary_detection_array.header.stamp > latest_tf)
//        {
//          //return if not
//          return -1;
//        }

        try
        {
          tf_listener_.lookupTransform("/map",temporary_detection_array.detections[it].pose.header.frame_id,temporary_detection_array.header.stamp,transform_li_);
        }
        catch (tf::TransformException ex)
        {
          ROS_ERROR("Skipping this detection. %s",ex.what());
          //detection_temp_storage_.pop();
          continue;
        }
        catch (tf::ExtrapolationException ex)
        {
          ROS_ERROR("Skipping this detection. %s",ex.what());
          //detection_temp_storage_.pop();
          continue;
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
      classifyDetections( temporary_detection_array);
      detection_temp_storage_.pop();
    }
    //publish the new array
    pub_all_recognitions_.publish(all_detections_array_);

    return 0;
}

/*! All the internally used map have to fit to the one static map coming from the /map topic. This callback should just be called once. If the mapserver has autosend on, this implementation will fail. The advantage of this implementation is, that it automatically adapts if a new map is loaded.*/

void person_detector_class::mapCallback_(const nav_msgs::OccupancyGrid received_map)
{
  //initialize map
  static_map_.resizeMap(received_map.info.width,received_map.info.height,received_map.info.resolution,received_map.info.origin.position.x,received_map.info.origin.position.y);
  updated_map_.resizeMap(received_map.info.width,received_map.info.height,received_map.info.resolution,received_map.info.origin.position.x,received_map.info.origin.position.y);
  difference_map_.resizeMap(received_map.info.width,received_map.info.height,received_map.info.resolution,received_map.info.origin.position.x,received_map.info.origin.position.y);
  updated_dm_.resizeMap(received_map.info.width,received_map.info.height,received_map.info.resolution,received_map.info.origin.position.x,received_map.info.origin.position.y);
  updated_counter_.resizeMap(received_map.info.width,received_map.info.height,received_map.info.resolution,received_map.info.origin.position.x,received_map.info.origin.position.y);
  dmap_new_.resizeMap(received_map.info.width,received_map.info.height,received_map.info.resolution,received_map.info.origin.position.x,received_map.info.origin.position.y);
  dmap_pano_.resizeMap(received_map.info.width,received_map.info.height,received_map.info.resolution,received_map.info.origin.position.x,received_map.info.origin.position.y);
  //set costs for static map and reset the updated map
  std::vector<signed char, std::allocator<signed char> >::const_iterator id = received_map.data.begin();
  int i = received_map.data.size();
  for (int ih = 0; ih < received_map.info.height; ih++)
  {
      for (unsigned int iw = 0; iw < received_map.info.width; iw++)
        {
          //for some reasons, we have to do the cast ourselves
          if (*id == -1) static_map_.setCost(iw,ih,costmap_2d::NO_INFORMATION);
          else if (*id == 100) static_map_.setCost(iw,ih,costmap_2d::LETHAL_OBSTACLE);
          else static_map_.setCost(iw,ih,costmap_2d::FREE_SPACE);
          updated_map_.setCost(iw,ih,costmap_2d::NO_INFORMATION);
          updated_dm_.setCost(iw,ih,254);
          id++;
        }
  }
  //publish static map
  std::string map_name = "person_detector/static_map";
  std::string map_frame = received_map.header.frame_id;
  pub_static_map_ = new costmap_2d::Costmap2DPublisher(&n_,&static_map_,map_frame,map_name,false);
  map_name = "person_detector/new_obstacles";
  pub_dmap_new_ = new costmap_2d::Costmap2DPublisher(&n_,&dmap_new_,map_frame,map_name,true);
  map_name = "person_detector/pano_map";
  pub_dmap_pano_ = new costmap_2d::Costmap2DPublisher(&n_,&dmap_pano_,map_frame,map_name,true);
  inflateMap();
  pub_static_map_->publishCostmap();
  pub_updated_map_->publishCostmap();
  pub_difference_map_->publishCostmap();
  ROS_INFO("Map received - start working");
  map_initialized_ = true;
}

/*! Takes the localCostmap from the move_base node and updates the updated_map with the information. This is the way to get rid of false detected obstacles marked by the obstacleCallback_, because the localCostmap also provides FREE_SPACE. Every occupied point gets one additional hit on the updated_counter_ map and gets a minumum distance of 1m on the updated_dm_ map. A FREE_SPACE substracts 10 points from updated_counter_. If an occupied point has less than 10 points left, it is going to be marked as FREE_SPACE in the updated_map_. This behaviour should make the updated_map_ a bit more stable.*/

void person_detector_class::localCostmapCallback_(const nav_msgs::OccupancyGrid received)
{
  //exit, if we are not ready to receive updates
  if (!map_initialized_) return;

  //integrate result into updated_map
  double x_diff = received.info.origin.position.x - updated_map_.getOriginX();
  double y_diff = received.info.origin.position.y - updated_map_.getOriginY();
  double map_res = updated_map_.getResolution();
  double rec_res = received.info.resolution;
  unsigned int point_map_x = x_diff / map_res;
  unsigned int point_map_y = y_diff / map_res;
  unsigned int rows = 0;
  int state = 0;
  //for (std::vector<signed char, std::allocator<signed char> >::const_iterator it = received.data.begin(); it != received.data.end(); it++)
  std::vector<signed char, std::allocator<signed char> >::const_iterator it = received.data.begin();
  for (unsigned int row = 0; row < received.info.height; row++)
    {
      for (unsigned col = 0; col < (received.info.width); col++)
        {
          int counter = updated_counter_.getCost(point_map_x,point_map_y);
          if (*it == 100)  // is occupied
          {
            updated_map_.setCost(point_map_x,point_map_y,costmap_2d::LETHAL_OBSTACLE);
            if (counter < 255)
            {
              updated_counter_.setCost(point_map_x,point_map_y,counter+1);
            }
          }
          else  //is free
          {
            if (counter > 10)
            {
              updated_counter_.setCost(point_map_x,point_map_y,counter-10);
            }
            else
            {
              updated_counter_.setCost(point_map_x,point_map_y,0);
              updated_map_.setCost(point_map_x,point_map_y,costmap_2d::FREE_SPACE);
            }
          }
          //update distances
          updated_dm_.setCost(point_map_x,point_map_y,10);
          //update point
          point_map_x = (x_diff + ((col+1)*rec_res)) / map_res;
          it++;
        }
      rows++;
      point_map_y = (y_diff + (row+1)*rec_res) / map_res;
      //reset point_map_x
      point_map_x = (x_diff / map_res);
  }
//  pub_updated_map->publishCostmap();
}

/*! Calculates the distance for each point. As the sensor data is too noisy on large distances and when the robot is turning, the function exits if the robot has an angular z velocity (turning) and scraps results being more than 4m away. If a point fulfills the requirements, it is set as LETHAL_OBSTACLE on the updated_map_.
    \todo Make maximum distance easier to set */

void person_detector_class::obstaclesCallback_(const sensor_msgs::PointCloud pcl)
{
  //we don't do anything if the map is not initialized
  if (!map_initialized_) return;
  //initialize helper variables
  double map_orig_x = updated_map_.getOriginX();
  double map_orig_y = updated_map_.getOriginY();
  double map_res = updated_map_.getResolution();
  double x_diff;
  double y_diff;
  double x_diff2;
  double y_diff2;
  double distance_m;
  int distance_dm;
  int point_x;
  int point_y;
  geometry_msgs::PoseWithCovarianceStamped pose;
  //find the best fitting amcl_pose of the robot
  findAmclPose(pose,pcl.header.stamp);
  double x_lat = pose.pose.pose.position.x;
  double y_lat = pose.pose.pose.position.y;
  //throw away data produced while turning
  if (imu_ang_vel_z != 0) return; //the data is useless, if we turn

  for (std::vector<geometry_msgs::Point32>::const_iterator it = pcl.points.begin(); it != pcl.points.end(); it++)
    {
      //calculate difference
      x_diff2 = (it->x -x_lat)*(it->x - x_lat);
      y_diff2 = (it->y - y_lat)*(it->y - y_lat);
      distance_m = sqrt(x_diff2+y_diff2);
      //throw away results being more than 4m away
      if (distance_m > 4) continue;
      //find the points
      x_diff = it->x - map_orig_x;
      y_diff = it->y - map_orig_y;
      point_x = x_diff / map_res;
      point_y = y_diff / map_res;
      //update point
      updated_map_.setCost(point_x,point_y,costmap_2d::LETHAL_OBSTACLE);
      distance_dm = round(distance_m*10);
      if (distance_m < updated_dm_.getCost(point_x,point_y))
      {
        updated_dm_.setCost(point_x,point_y,distance_dm);
      }
      int counter = updated_counter_.getCost(point_x,point_y);
      if (counter < 254)
        {
          updated_counter_.setCost(point_x,point_y,counter+1);
        }
    }
}

/*! The gyrometer data is needed by the obstaceCallback_ to discard points if the robot is turning.*/

void person_detector_class::imuCallback_(const sensor_msgs::Imu imu)
{
  imu_ang_vel_z = imu.angular_velocity.z;
}

/*! This function just saves the confirmations in a queue. */

void person_detector_class::confirmationCallback_(const person_detector::SpeechConfirmation conf)
{
  conf_queue_.push(conf);
}

/*! This function saves the latest 30 amcl poses in a vector.*/

void person_detector_class::amclCallback_(const geometry_msgs::PoseWithCovarianceStamped pose)
{
  amcl_poses_.push_back(pose);
  if (amcl_poses_.size() > 30)
  {
    amcl_poses_.erase(amcl_poses_.begin());
  }
}

/*! This function akes all incoming detections and decides if they fulfill the requirements to update a known detection or if they are going to be added as a new detection. The distances between all incoming and known detection are calculated and the incoming detection will update a known detection if it is nearer than a specified distance (at the moment 50cm). This function also cares about resolving an situation when a few incoming detections are supposed to update the same known detection. In that case the closest hit will be used to update and the other one will be added as a new detection.
    \todo Make maximum distance easier to set.
    \todo Function always return true - can it never fail?
    \sa processDetections \sa updateDetection \sa addNewDetection \sa clearDoubleResults */

int person_detector_class::classifyDetections( cob_people_detection_msgs::DetectionArray detection_array )
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
  //try to find a corresponging match within a distance of x m
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

  //find the winner
  ROS_DEBUG("Searching for the winner");
  std::vector<unsigned int> win_id (detection_array.detections.size());
  std::vector<double> win_dist (detection_array.detections.size());
  //populate vector with incredible high values
  for (unsigned int in = 0; in < detection_array.detections.size(); in++)
  {
      win_dist[in] = 1000000;
  }
  findDistanceWinner(distances,win_id,win_dist,detection_array.detections.size());
  //check for double results
  clearDoubleResults(distances,win_id,win_dist,detection_array.detections.size());

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

/*! Initializes a new detection and adds it to the detection storage and assigns the first namelabel to the detection.
    \sa updateDetection */

int person_detector_class::addNewDetection(cob_people_detection_msgs::Detection new_detection)
{
  ROS_INFO("Addind a new detection");
  person_detector::DetectionObject push_object;
  person_detector::NameLabel push_name;

  push_object.header.stamp = push_object.latest_pose_map.header.stamp = new_detection.header.stamp;  //why not taking the stamp of the object
  push_object.header.seq = recognition_id_;
  recognition_id_++;
  push_object.latest_pose_map.header.frame_id = "/map";
  push_object.total_detections = 1;
  push_object.latest_pose_map.pose.position.x = new_detection.pose.pose.position.x;
  push_object.latest_pose_map.pose.position.y = new_detection.pose.pose.position.y;
  push_object.latest_pose_map.pose.position.z = new_detection.pose.pose.position.z;
  push_object.confirmation.label = "";
  push_object.confirmation.running = false;
  push_object.confirmation.suceeded = false;
  push_object.confirmation.tried = false;
  findAmclPose(push_object.last_seen_from,new_detection.header.stamp);
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

/*! Updates the position of a known detection with information of an incoming detection. The pose of the known detection is set to the one of the incoming detection and the name assigned to the detection is updated.
    \sa addNewDetection*/

int person_detector_class::updateDetection(cob_people_detection_msgs::Detection new_detection, unsigned int pos)
{
  ROS_DEBUG("Updating a detection");
  //update general stuff
  all_detections_array_.header.stamp = ros::Time::now();
  all_detections_array_.header.seq++;

  //update our specific result
  all_detections_array_.detections[pos].total_detections++;
  all_detections_array_.detections[pos].latest_pose_map.pose.position.x = new_detection.pose.pose.position.x;
  all_detections_array_.detections[pos].latest_pose_map.pose.position.y = new_detection.pose.pose.position.y;
  all_detections_array_.detections[pos].latest_pose_map.pose.position.z = new_detection.pose.pose.position.z;
  all_detections_array_.detections[pos].latest_pose_map.header.stamp = ros::Time::now();
  findAmclPose(all_detections_array_.detections[pos].last_seen_from,new_detection.header.stamp);
  //update or create a new nametag
  bool found = false;
  std::string it_label;
  if (new_detection.label != "UnknownHead" && new_detection.label != "Unknown")
  {
    for (unsigned int it = 0; it < all_detections_array_.detections[pos].recognitions.name_array.size(); it++)
    {
      it_label = all_detections_array_.detections[pos].recognitions.name_array[it].label;
      if (new_detection.label == all_detections_array_.detections[pos].recognitions.name_array[it].label)
      {
          all_detections_array_.detections[pos].recognitions.name_array[it].quantity++;
          all_detections_array_.detections[pos].recognitions.total_assigned++;
          found = true;
          ROS_DEBUG("Found a hit and now substracting of the rest");
          substractHit(new_detection.label, pos);
      }
    }
    if (!found)
      {
        person_detector::NameLabel push_name;
        push_name.label = new_detection.label;
        push_name.quantity = 1;
        all_detections_array_.detections[pos].recognitions.name_array.push_back(push_name);
        all_detections_array_.detections[pos].recognitions.total_assigned++;
        substractHit(new_detection.label, pos);
      }
  }
  return 0;
}

/*! Helper function for clearDoubleResults_ and classifyDetections_. After the distances between known and incoming detections have been found, this function finds the closest known detection of an incoming detection and saves it in win_id and win_dist.
    \sa classifyDetections \sa clearDoubleResults */

int person_detector_class::findDistanceWinner(std::vector< std::vector <double> > &distances, std::vector<unsigned int> &win_id, std::vector<double> &win_dist, unsigned int detection_array_size)
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

/*! E.g. in the case that we have 1 known detection, but 2 incoming detection both incoming detections will have the known detection as nearest known detection. This function resolves this conflict, as it's now possible to assign both of them to the same known detection. This function exits if all known or all incoming detections have been assigned.
    \sa classifyDetections */

int person_detector_class::clearDoubleResults(std::vector< std::vector <double> > &distances, std::vector<unsigned int> &win_id, std::vector<double> &win_dist, unsigned int detection_array_size)
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

  //safety exit from the loop
  ros::Time start = ros::Time::now();
  ros::Duration max_time = ros::Duration(1.5);
  while (ros::ok())
  {
    if ((ros::Time::now()-start) > max_time)
    {
      ROS_WARN("Safety exit from the clear double result. This should not happen!");
      return -1;
    }
    //searching for the shortest distance first
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
    findDistanceWinner(distances,win_id,win_dist,win_id.size());
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

/*! Whenever we detection a person with the name <Example> it is unlikey that this person is anywhere else on the map. So every other known detection of the map having the name label <Example> gets substracted one hit on that name. If the counter for the tag gets 0 the name tag will be deleted.
    \sa updateDetections \sa addNewDetection */

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

/*! Face detections without any names as well as unpresent obstacles don't get deleted immediately. This is done by the garbage collector erasing all face detections without names assigned and unpresent obstacle which haven't been updated for a longer time than specified in the parameter. */

int person_detector_class::garbageCollector(ros::Duration oldness)
{
  ros::Time present = ros::Time::now();
  ros::Time time_stamp;
  if (!all_detections_array_.detections.empty())
  {
    for (unsigned int it = 0; it < all_detections_array_.detections.size(); it++)
    {
        time_stamp = all_detections_array_.detections[it].latest_pose_map.header.stamp;
        if(all_detections_array_.detections[it].recognitions.name_array.empty() && ((present - time_stamp) > oldness))
          {
            all_detections_array_.detections.erase(all_detections_array_.detections.begin()+it);
          }
    }
  }
  if (!all_obstacles_.obstacles.empty())
  {
    for (unsigned int it = 0; it < all_obstacles_.obstacles.size(); it++)
    {
      if (all_obstacles_.obstacles[it].present == false &&
          all_obstacles_.obstacles[it].confirmation.suceeded == false &&
          all_obstacles_.obstacles[it].confirmation.tried == false &&
          all_obstacles_.obstacles[it].confirmation.running == false)
      {
        time_stamp = all_obstacles_.obstacles[it].header.stamp;
        if ((present - time_stamp) > oldness)
        {
            all_obstacles_.obstacles.erase(all_obstacles_.obstacles.begin()+it);
            all_obs_map_xy_.erase(all_obs_map_xy_.begin()+it);
        }
      }
    }
  }
}

/*! This functions searches for LETHAL_OBSTACLES in the given map and marks all points around them as occupied. */

int person_detector_class::inflateMap()
{
  std::vector<unsigned int> lethal_ix;
  std::vector<unsigned int> lethal_iy;
  unsigned char cost = costmap_2d::LETHAL_OBSTACLE;
  for(unsigned int j = 0; j < static_map_.getSizeInCellsY(); j++)
    {
        for(unsigned int i = 0; i < static_map_.getSizeInCellsX(); i++)
          {
        cost = static_map_.getCost(i,j);
        if(static_map_.getCost(i, j) == costmap_2d::LETHAL_OBSTACLE)
        {
          //add to vector
          lethal_ix.push_back(i);
          lethal_iy.push_back(j);
        }
    }
  }
  unsigned int lat_x;
  unsigned int lat_y;
  //now that we found all lethal points, we can inflate
  while (!lethal_ix.empty() || !lethal_iy.empty())
  {
    lat_x = lethal_ix.back();
    lat_y = lethal_iy.back();
    for (int ix = -3; ix < 4; ix++)
      {
        for (int iy = -3; iy < 4; iy++)
          {
            static_map_.setCost(lat_x-ix,lat_y-iy,costmap_2d::LETHAL_OBSTACLE);
          }
      }
    lethal_ix.pop_back();
    lethal_iy.pop_back();
  }
  return 0;
}

/*! Generates the difference between the infalted static map and the updated_map_ to show which points are additionally occupied.
    \sa difference_map_ \sa updated_map_ */

int person_detector_class::generateDifferenceMap()
{
  for (unsigned int ix = 0; ix < static_map_.getSizeInCellsX(); ix++)
    {
      for (unsigned int iy = 0; iy < static_map_.getSizeInCellsY(); iy++)
        {
          if (updated_map_.getCost(ix,iy) == costmap_2d::LETHAL_OBSTACLE && static_map_.getCost(ix,iy) == costmap_2d::FREE_SPACE)
            {
              difference_map_.setCost(ix,iy,costmap_2d::LETHAL_OBSTACLE);
            }
          else
            {
              difference_map_.setCost(ix,iy,costmap_2d::FREE_SPACE);
            }
        }
    }
  return 0;
}

/*! This function first tries to find all known obstacles on the latest difference_map_. If more than 5 points are found, the known obstacle will get updated. Otherwise it is marked as not present. The rest of the occupied points are added as new obsacles if more than 7 points are connected.
    \sa all_obstacles_ */

int person_detector_class::findObstacles()
{
  if (!map_initialized_) return 0;
  //initialize with difference map
  all_obstacles_.header.seq++;
  all_obstacles_.header.stamp = ros::Time::now();
  costmap_2d::Costmap2D new_map;
  new_map = difference_map_;
  //if we already have some, try to find them
  if (!all_obstacles_.obstacles.empty())
  {
    std::vector<geometry_msgs::Point> tmp_p;
    std::vector<geometry_msgs::Point> tmp_p_map_xy;
    for (unsigned int it = 0; it < all_obstacles_.obstacles.size(); it++)
    {
      tmp_p.clear();
      tmp_p_map_xy.clear();
      //try to refind the old points
      for (unsigned int ip = 0; ip < all_obs_map_xy_[it].points.size(); ip++)
      {
        if (new_map.getCost(all_obs_map_xy_[it].points[ip].x,all_obs_map_xy_[it].points[ip].y) == costmap_2d::LETHAL_OBSTACLE)
        {
          searchFurther(all_obs_map_xy_[it].points[ip].x,all_obs_map_xy_[it].points[ip].y,&new_map,&tmp_p,&tmp_p_map_xy);
        }
      }
      //obstacles smaller than 5 points are rejected
      if (tmp_p.size() < 5)
      {
        all_obstacles_.obstacles[it].present = false;
      }
      else
      {
        //update points
        all_obstacles_.obstacles[it].points = tmp_p;
        all_obs_map_xy_[it].points = tmp_p_map_xy;
        //update time
        all_obstacles_.obstacles[it].header.stamp = ros::Time::now();
        all_obstacles_.obstacles[it].present = true;
        rateObstacle_(&all_obstacles_.obstacles[it],&all_obs_map_xy_[it]);
      }

    }
  }

  //now find the new ones
  person_detector::Obstacle obs;
  person_detector::ObsMapPoints obs_map_xy;
  obs.header.frame_id = "/map";
  obs.header.stamp = ros::Time::now();
  for (unsigned int ix = 0;ix < new_map.getSizeInCellsX(); ix++)
  {
    for (unsigned int iy = 0; iy < new_map.getSizeInCellsY(); iy++)
    {
      if (new_map.getCost(ix,iy) == costmap_2d::LETHAL_OBSTACLE)
        {
          obs_map_xy.points.clear();
          obs.points.clear();
          new_map.setCost(ix,iy,costmap_2d::FREE_SPACE);
          new_map.mapToWorld(ix,iy,x_map_,y_map_);
          p_map_xy_.x = ix;
          p_map_xy_.y = iy;
          p_.x = x_map_;
          p_.y = y_map_;
          p_.z = 0;
          obs.points.push_back(p_);
          obs_map_xy.points.push_back(p_map_xy_);
          searchFurther(ix,iy,&new_map,&obs.points,&obs_map_xy.points);
          //we discard every obstacle smaller than 7 points
          if (obs.points.size() > 7)
          {
            rateObstacle_(&obs,&obs_map_xy);
            findAmclPose(obs.robot_pose,ros::Time::now());
            obs.header.seq = recognition_id_;
            obs.present = true;
            recognition_id_++;
            all_obstacles_.obstacles.push_back(obs);
            all_obs_map_xy_.push_back(obs_map_xy);
          }
        }
    }
  }
  //publish new array
  all_obstacles_.header.seq++;
  all_obstacles_.header.stamp = ros::Time::now();
  pub_all_obstacles_.publish(all_obstacles_);
  return 0;
}

bool person_detector_class::searchFurther(unsigned int x_orig, unsigned int y_orig, costmap_2d::Costmap2D *costmap, std::vector<geometry_msgs::Point> *points, std::vector<geometry_msgs::Point> *points_map_xy)
{
  for (int ix = -3; ix < 4; ix++)
  {
    for (int iy = -3; iy < 4; iy++)
    {
      if (costmap->getCost(x_orig+ix,y_orig+iy) == costmap_2d::LETHAL_OBSTACLE)
        {
          costmap->setCost(x_orig+ix,y_orig+iy,costmap_2d::FREE_SPACE);
          costmap->mapToWorld(x_orig+ix,y_orig+iy,x_map_,y_map_);
          p_map_xy_.x = x_orig+ix;
          p_map_xy_.y = y_orig+iy;
          p_.x = x_map_;
          p_.y = y_map_;
          p_.z = 0;
          points->push_back(p_);
          points_map_xy->push_back(p_map_xy_);
          searchFurther(x_orig+ix,y_orig+iy,costmap,points,points_map_xy);
        }
    }
  }
}

/*! This function rates an obstacle based on its size, the number of appearances of the points and the mean distance from which the points have been seen. At the moment the total score is equally partitioned between these 3 features. This means 33 of the 100 total points are influenced by the size, 33 by the distance and 33 by the number of appearances.*/

bool person_detector_class::rateObstacle_(person_detector::Obstacle *obs, person_detector::ObsMapPoints *map_points)
{
  //right now, we calculate an average
  int total_points = 0;
  double rate_counts = 0;
  double rate_range = 0;
  double point_range = 0;
  double rate_form = 0;
  //how often have we seen this obstacle in average?
  //from which distance have we seen this obstacle?
  //
  for (unsigned int it = 0; it < map_points->points.size(); it++)
  {
      rate_counts += updated_counter_.getCost(map_points->points[it].x,map_points->points[it].y);
      point_range = updated_dm_.getCost(map_points->points[it].x,map_points->points[it].y);
      // our accepted distances vary from 10dm to 40dm
      // a 10dm distance gives 100 points
      // 40dm and further give 0 points
      // this leads to points = distance_in_dm * (-10/3) + 400/3
      if (point_range < 40)
      {
          rate_range += point_range * (-10/3) + 400/3;
      }
  }
  //build the average of counts and normalize it to 100
  rate_counts = rate_counts/(map_points->points.size()*255);
  //build the average of the range rate
  rate_range = rate_range/map_points->points.size();

  //rate of the form
  if (map_points->points.size() < 5)
  {
    rate_form = 10;
  }
  else if (map_points->points.size() < 10)
  {
    rate_form = 50;
  }
  else if (map_points->points.size() < 20)
  {
    rate_form = 75;
  }
  else
  {
    rate_form = 100;
  }

  //total rate
  total_points = (rate_counts + rate_range + rate_form)/3;
  obs->probability = total_points;
  return true;
}

/*! This function shows all obstacle information on rviz. The first kind of information is a short information string to every occupied point. It is displayed in the way "[number of appearances] | [shortest distance in dm]. The second information is a line connecting all points of an obstacle. The third is a cube representing the center of an obstacle. The fourth is an infotext displayed at the center of the obstacle and showing the ID of the obstacle, its rating and if it is confirmed. */

void person_detector_class::showAllObstacles()
{
  //nothing to do, if we're not initialized
  if (!map_initialized_) return;
  //display text to every occupied point
  if (pub_obstacle_points_text_.getNumSubscribers() > 0)
    {
      obstacle_points_text_.header.stamp = ros::Time::now();
      int id = 0;;
      double map_x;
      double map_y;
      std::string name;
      visualization_msgs::MarkerArray text_array;
      for (unsigned int ix = 0; ix < updated_map_.getSizeInCellsX(); ix++)
      {
        for (unsigned int iy = 0; iy < updated_map_.getSizeInCellsY(); iy++)
        {
            if (difference_map_.getCost(ix,iy) == costmap_2d::LETHAL_OBSTACLE)
          {
              obstacle_points_text_.id = id;
              updated_dm_.mapToWorld(ix,iy+1,map_x,map_y);
              obstacle_points_text_.pose.position.x = map_x;
              obstacle_points_text_.pose.position.y = map_y;
              obstacle_points_text_.pose.position.z = 0.05;
              //data.push_back(updated_counter_.getCost(ix,iy));
              int count = updated_counter_.getCost(ix,iy);
              name = boost::lexical_cast<std::string>(count);
              name += "|";
              int dm = updated_dm_.getCost(ix,iy);
              name+= boost::lexical_cast<std::string>(dm);
              obstacle_points_text_.text = name;
              text_array.markers.push_back(obstacle_points_text_);
              name.clear();
              id++;
          }
        }
      }
      pub_obstacle_points_text_.publish(text_array);
    }

  //display obstacle borders
  if (pub_obstacle_borders_.getNumSubscribers() > 0)
    {
      visualization_msgs::MarkerArray array;
      //define colors
      std_msgs::ColorRGBA color;
      color.g = 1.0;
      color.b = 0;
      color.r = 0;
      color.a = 1;
      for (unsigned int it = 0; it < all_obstacles_.obstacles.size(); it++)
        {
          obstacle_boarder_marker_.colors.clear();
          obstacle_boarder_marker_.points.clear();
          //first declare an white extra point
          obstacle_boarder_marker_.points.push_back(all_obstacles_.obstacles[it].points.front());
          obstacle_boarder_marker_.points.front().x = obstacle_boarder_marker_.points.front().x+0.04;
          color.b = 1;
          color.g = 1;
          color.r = 1;
          obstacle_boarder_marker_.colors.push_back(color);
          for (unsigned int ip = 0; ip < all_obstacles_.obstacles[it].points.size(); ip++)
          {
              obstacle_boarder_marker_.points.push_back(all_obstacles_.obstacles[it].points[ip]);
              color.b = 0;
              color.g = 1;
              color.r = 0;
              obstacle_boarder_marker_.colors.push_back(color);
          }
          //obstacle_marker_.points = all_obstacles_.obstacles[it].points;
          //add the first point to close the circle
          obstacle_boarder_marker_.points.push_back(obstacle_boarder_marker_.points.front());
          obstacle_boarder_marker_.points.back().x = obstacle_boarder_marker_.points.back().x-0.04;
          obstacle_boarder_marker_.colors.push_back(color);
          color.b = 1;
          color.g = 1;
          color.r = 1;
          //add an additional white point at the end
          obstacle_boarder_marker_.points.push_back(obstacle_boarder_marker_.points.back());
          obstacle_boarder_marker_.points.back().y = obstacle_boarder_marker_.points.back().y-0.04;
          obstacle_boarder_marker_.colors.push_back(color);
          //update header
          obstacle_boarder_marker_.id = all_obstacles_.obstacles[it].header.seq;
          obstacle_boarder_marker_.header.seq = all_obstacles_.obstacles[it].header.seq;
          obstacle_boarder_marker_.header.stamp = ros::Time::now();
          array.markers.push_back(obstacle_boarder_marker_);


        }
      pub_obstacle_borders_.publish(array);
    }
  //show obstacle cubes and obstacle info-text
  if (pub_obstacle_cubes_.getNumSubscribers() > 0 || pub_obstacle_info_text_.getNumSubscribers() > 0)
  {
    //helper variables
    double x;
    double y;
    geometry_msgs::Point p;
    visualization_msgs::MarkerArray text_array;
    visualization_msgs::MarkerArray cube_array;
    std::string text;
    p.z = 0.2;
    for (unsigned int it = 0; it < all_obstacles_.obstacles.size(); it++)
    {
      //calculate center
      x = 0;
      y = 0;
      for (unsigned int ip = 0; ip < all_obstacles_.obstacles[it].points.size(); ip++)
      {
        x += all_obstacles_.obstacles[it].points[ip].x;
        y += all_obstacles_.obstacles[it].points[ip].y;
      }
      x = x / all_obstacles_.obstacles[it].points.size();
      y = y / all_obstacles_.obstacles[it].points.size();
      //assign value
      obstacle_info_text_.pose.position.x = p.x = x;
      p.y = y;
      obstacle_info_text_.pose.position.y = y;
      obstacle_cubes_.points.clear();
      obstacle_cubes_.points.push_back(p);
      //build text
      text.clear();
      text += "ID:";
      text += boost::lexical_cast<std::string>(all_obstacles_.obstacles[it].header.seq);
      text += " | ";
      int prob = all_obstacles_.obstacles[it].probability;
      text += boost::lexical_cast<std::string>(prob);
      text += "p";
      if (all_obstacles_.obstacles[it].confirmation.running)
      {
          text += " | running";
      }
      if (all_obstacles_.obstacles[it].confirmation.tried)
      {
        text += " | tried";
      }
      if (all_obstacles_.obstacles[it].confirmation.suceeded)
      {
        if (all_obstacles_.obstacles[it].confirmation.label.empty())
        {
          text += " | human";
        }
        else
        {
          text += " | ";
          text += all_obstacles_.obstacles[it].confirmation.label;
        }
      }
      obstacle_info_text_.text = text;
      //change color in order of their state
      obstacle_info_text_.color.r = obstacle_info_text_.color.a = 1;
      obstacle_info_text_.color.b = obstacle_info_text_.color.g = 0;
      // green if checked
      if (all_obstacles_.obstacles[it].confirmation.suceeded)
      {
        obstacle_cubes_.color.b = 0;
        obstacle_cubes_.color.g = 1.0;
        obstacle_cubes_.color.r = 0;
      }
      // yellow if running or tried
      else if (all_obstacles_.obstacles[it].confirmation.tried || all_obstacles_.obstacles[it].confirmation.running)
      {
        obstacle_cubes_.color.b = 0;
        obstacle_cubes_.color.g = 1;
        obstacle_cubes_.color.r = 1;
      }
      // present obstacles will be blue
      if (all_obstacles_.obstacles[it].present)
      {
        obstacle_cubes_.color.b = 1.0;
        obstacle_cubes_.color.g = 0;
        obstacle_cubes_.color.r = 0;
      }
      // all inactive, but not yet garbage-collected obstacles are greyish
      else
      {
      obstacle_cubes_.color.r = 0.8;
      obstacle_cubes_.color.g = 0.8;
      obstacle_cubes_.color.b = 0.8;
      obstacle_info_text_.color.r = 0.8;
      obstacle_info_text_.color.b = 0.8;
      obstacle_info_text_.color.g = 0.8;
      }

      //update headers
      obstacle_cubes_.id = obstacle_info_text_.id = all_obstacles_.obstacles[it].header.seq;
      obstacle_cubes_.header.seq++;
      obstacle_info_text_.header.seq++;
      obstacle_cubes_.header.stamp = obstacle_info_text_.header.stamp = ros::Time::now();
      //add to array
      text_array.markers.push_back(obstacle_info_text_);
      cube_array.markers.push_back(obstacle_cubes_);
    }
    pub_obstacle_info_text_.publish(text_array);
    pub_obstacle_cubes_.publish(cube_array);
  }
}

/*! This function updates the obstacles and face recognition with information from an external confirmation source. */

int person_detector_class::processConfirmations()
{
  person_detector::SpeechConfirmation act;
  while (!conf_queue_.empty())
  {
    act = conf_queue_.front();
    conf_queue_.pop();
    bool found = false;
    //search for id in obstacles
    for (unsigned int it = 0; it < all_obstacles_.obstacles.size(); it++)
    {
        //if we find the right one, we save the information an
        if (all_obstacles_.obstacles[it].header.seq == act.id)
        {
            all_obstacles_.obstacles[it].confirmation = act;
            found = true;
            break;
        }
    }
    // we can skip the second part, if we've found it
    if (found) continue;
    //search for id in face recognitions
    for (unsigned int it = 0; it < all_detections_array_.detections.size(); it++)
    {
        // if it's the right, put information in there
        if (all_detections_array_.detections[it].header.seq == act.id)
        {
            all_detections_array_.detections[it].confirmation = act;
            break;
        }
    }
    //if we were successfull, delete other detections of that person
    if (act.suceeded)
    {
      for (unsigned int it = 0; it < all_detections_array_.detections.size(); it++)
      {
        if (all_detections_array_.detections[it].header.seq != act.id)
          {
            //go through all name_array
            for (unsigned int in = 0; in < all_detections_array_.detections[it].recognitions.name_array.size(); in++)
            {
              if (act.label == all_detections_array_.detections[it].recognitions.name_array[in].label)
              {
                  all_detections_array_.detections[it].recognitions.name_array.erase(all_detections_array_.detections[it].recognitions.name_array.begin()+in);
              }
            }
          }
      }
    }
  }
}

/*! Sometimes it is necessary to known the robots position of a certain moment in the past. This function finds the best fitting pose.*/

bool person_detector_class::findAmclPose(geometry_msgs::PoseWithCovarianceStamped &pose, ros::Time stamp)
{
  double t_tmp_diff = 0;
  double t_diff = 1000;
  unsigned int winner = 0;
  if (amcl_poses_.empty())
    {
      ROS_ERROR("We couldn't find an amcl pose fitting to that - this should not happen!");
      return false;
    }
  for (unsigned int it = 0; it < amcl_poses_.size(); it++)
  {
    t_tmp_diff = std::abs((amcl_poses_[it].header.stamp - stamp).toSec());
    if (t_tmp_diff < t_diff )
    {
      t_diff = t_tmp_diff;
      winner = it;
    }
  }
  pose = amcl_poses_[winner];
  return true;
}

person_detector_class::person_detector_class()
{
  //initialize ros
  sub_face_recognition_ = n_.subscribe("/cob_people_detection/detection_tracker/face_position_array",10, &person_detector_class::faceRecognitionCallback_,this);
  sub_imu_ = n_.subscribe("/mobile_base/sensors/imu_data",10,&person_detector_class::imuCallback_,this);
  sub_confirmations_ = n_.subscribe("/person_detector/confirmations",10,&person_detector_class::confirmationCallback_,this);
  sub_amcl_ = n_.subscribe("/amcl_pose",10,&person_detector_class::amclCallback_,this);
  tf_cache_ = tf_listener_.getCacheLength();
  pub_all_recognitions_ = n_.advertise<person_detector::DetectionObjectArray>("/person_detector/all_recognitions",10);
  pub_all_obstacles_ = n_.advertise<person_detector::ObstacleArray>("/person_detector/all_obstacles",10);
  //initialize markers
  pub_human_marker_raw_ = n_.advertise<visualization_msgs::Marker>("/person_detector/face_marker_raw",10);
  pub_human_marker_raw_text_ = n_.advertise<visualization_msgs::Marker>("/person_detector/face_marker_text_raw",10);
  pub_human_marker_ = n_.advertise<visualization_msgs::Marker>("/person_detector/human_marker",10);
  pub_human_marker_text_ = n_.advertise<visualization_msgs::Marker>("/person_detector/human_marker_text",10);
  pub_obstacle_points_text_ = n_.advertise<visualization_msgs::MarkerArray>("/person_detector/obstacle_appearance_text",10);
  pub_obstacle_borders_ = n_.advertise<visualization_msgs::MarkerArray>("/person_detector/obstacle_borders",10);
  pub_obstacle_info_text_ = n_.advertise<visualization_msgs::MarkerArray>("/person_detector/obstacle_info_text",10);
  pub_obstacle_cubes_ = n_.advertise<visualization_msgs::MarkerArray>("/person_detector/obstacle_cubes",10);
  heads_raw.header.frame_id = "/camera_rgb_optical_frame";
  heads_raw.ns = "person_detector/face_marker";
  heads_raw.id = 0;
  heads_raw.lifetime = ros::Duration(10);
  heads_raw.action = visualization_msgs::Marker::ADD;
  heads_raw.type = visualization_msgs::Marker::POINTS;
  heads_raw.scale.x = 0.2;
  heads_raw.scale.y = 0.2;
  heads_raw.scale.z = 0.2;
  heads_raw.color.g = 1.0f;
  heads_raw.color.a = 1.0;

  text_raw_.header.frame_id = "/camera_rgb_optical_frame";
  text_raw_.ns = "person_detector/face_marker_text";
  text_raw_.id = 0;
  text_raw_.lifetime = ros::Duration(10);
  text_raw_.action = visualization_msgs::Marker::ADD;
  text_raw_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_raw_.scale.z = 0.2;
  text_raw_.color.r = 1.0f;
  text_raw_.color.a = 1.0;

  obstacle_points_text_.header.frame_id = "/map";
  obstacle_points_text_.ns = "person_detector/obstacle_text";
  obstacle_points_text_.id = 0;
  obstacle_points_text_.lifetime = ros::Duration(1);
  obstacle_points_text_.action = visualization_msgs::Marker::ADD;
  obstacle_points_text_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  obstacle_points_text_.scale.z = 0.02;
  obstacle_points_text_.color.r = 1.0f;
  obstacle_points_text_.color.a = 1.0;

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

  obstacle_boarder_marker_.action = visualization_msgs::Marker::ADD;
  obstacle_boarder_marker_.header.frame_id = "/map";
  obstacle_boarder_marker_.header.seq = 0;
  obstacle_boarder_marker_.header.stamp = ros::Time::now();
  obstacle_boarder_marker_.scale.x = 0.05;
  obstacle_boarder_marker_.scale.y = 0.05;
  obstacle_boarder_marker_.scale.z = 0.05;
  obstacle_boarder_marker_.color.r = 0;
  obstacle_boarder_marker_.color.g = 1;
  obstacle_boarder_marker_.color.b = 0;
  obstacle_boarder_marker_.color.a = 1;
  obstacle_boarder_marker_.ns = "person_detector/obstacle_marker";
  obstacle_boarder_marker_.type = visualization_msgs::Marker::LINE_STRIP;
  obstacle_boarder_marker_.lifetime = ros::Duration(1);

  obstacle_cubes_.action = visualization_msgs::Marker::ADD;
  obstacle_cubes_.header.frame_id = "/map";
  obstacle_cubes_.header.seq = 0;
  obstacle_cubes_.header.stamp = ros::Time::now();
  obstacle_cubes_.scale.x = obstacle_cubes_.scale.y = obstacle_cubes_.scale.z = 0.15;
  obstacle_cubes_.color.b = obstacle_cubes_.color.a = 1;
  obstacle_cubes_.color.g = obstacle_cubes_.color.r = 0;
  obstacle_cubes_.ns = "person_detector/obstacle_marker";
  obstacle_cubes_.type = visualization_msgs::Marker::POINTS;
  obstacle_cubes_.lifetime = ros::Duration(10);

  obstacle_info_text_.action = visualization_msgs::Marker::ADD;
  obstacle_info_text_.header.frame_id = "/map";
  obstacle_info_text_.header.seq = 0;
  obstacle_info_text_.header.stamp = ros::Time::now();
  obstacle_info_text_.scale.z = 0.1;
  obstacle_info_text_.pose.position.z = 0.5;
  obstacle_info_text_.color.r = obstacle_info_text_.color.a = 1;
  obstacle_info_text_.ns = "person_detector/obstacle_info_text";
  obstacle_info_text_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  obstacle_info_text_.lifetime = ros::Duration(10);


  //initialize array
  recognition_id_ = 0;

  //maps
  sub_map_ = n_.subscribe("/map",10,&person_detector_class::mapCallback_,this);
  sub_local_costmap_ = n_.subscribe("/move_base/local_costmap/costmap", 10, &person_detector_class::localCostmapCallback_,this);
  sub_obstacles_ = n_.subscribe("/move_base/global_costmap/obstacle_layer/clearing_endpoints",10,&person_detector_class::obstaclesCallback_,this);
  map_initialized_ = false;

  //updated_map
  updated_map_.resizeMap(10,10,1,0,0);
  updated_map_.updateOrigin(0,0);
  std::string map_frame = "map";
  std::string updated_map_name = "person_detector/updated_map";
  for (unsigned int ih = 0; ih < 10; ih++)
    {
      for (unsigned int iw = 0; iw < 5; iw++)
        {
          updated_map_.setCost(ih,iw,costmap_2d::NO_INFORMATION);
        }
    }
  pub_updated_map_ = new costmap_2d::Costmap2DPublisher(&n_,&updated_map_,map_frame,updated_map_name,true);
  pub_updated_map_->publishCostmap();
  //difference_map
  difference_map_.resizeMap(10,10,1,0,0);
  difference_map_.updateOrigin(0,0);
  std::string difference_map_name = "person_detector/difference_map";
  pub_difference_map_ = new costmap_2d::Costmap2DPublisher(&n_,&difference_map_,map_frame,difference_map_name,true);
  pub_difference_map_->publishCostmap();

  imu_ang_vel_z = 0;
}

/*! This is the main loop managing the whole process and running endless. After the object is initialized, this function should be called. */

int person_detector_class::run()
{
  ros::Rate r(15);
  ros::Time start;
  ros::Time end;
  ros::Duration difference;

  while (ros::ok())
  {
    start = ros::Time::now();
    processDetections();
    garbageCollector(ros::Duration(60));
    end = ros::Time::now();
    difference = end-start;
    processConfirmations();
    ROS_WARN_COND(detection_temp_storage_.size() > 10,"Our temporary storage is too big. It holds %i objects. Last circle took %f seconds.",detection_temp_storage_.size(),difference.toSec());
    r.sleep();
    ros::spinOnce();
    generateDifferenceMap();
    showAllRecognitions();
    findObstacles();
    showAllObstacles();
    pub_difference_map_->publishCostmap();
    pub_updated_map_->publishCostmap();
  }
}
