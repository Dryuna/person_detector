#ifndef PERSON_DETECTOR_H
#define PERSON_DETECTOR_H
#include <ros/ros.h>                                    // general ROS-functionalities
#include <cob_people_detection_msgs/DetectionArray.h>   // message type for the cob_people_detection_topic
#include <queue>                                        // used to store the detections
#include <vector>                                       // used to store amcl-poses
#include <tf/transform_listener.h>                      // currently unused
#include <tf/transform_broadcaster.h>                   // used to broadcast detections
#include <visualization_msgs/Marker.h>                  // display markers on rviz
#include <visualization_msgs/MarkerArray.h>             // advanced display of marker on rviz
#include <person_detector/DetectionObjectArray.h>       // our detections
#include <person_detector/DetectionObject.h>            // used for a single detection
#include <person_detector/SpeechConfirmation.h>         // for speech confirmations we receive
#include <person_detector/ObstacleArray.h>              // to store found obstacles
#include <nav_msgs/OccupancyGrid.h>                     // the map format
#include <costmap_2d/layer.h>                           // to use a costmap
#include <costmap_2d/costmap_2d_ros.h>                  // to use a costmap
#include <sensor_msgs/Imu.h>                            // to get information about the rotation
#include <geometry_msgs/PoseWithCovarianceStamped.h>    // for amcl

namespace person_detector {
  //! This struct is used to store the obstacle map points
  /*! The obstacle points in the Obstacle.msg are stored in metric map coordinates. This would require a lookup metric map coordinates to the corresponding fields in the costmap array each time the obstacle has to be found again. Therefore the arrays positions of all points of an obstacle are also stored in this struct.*/
  struct ObsMapPoints
  {
    unsigned int id;                                            //!< The unique ID of the obstacle
    std::vector<geometry_msgs::Point> points;                   //!< A vector points on the costmap array. They x and y values are pointing to a field on the costmap and aren't metric
  };
}

/*! The person_detector class manages the whole process of detections based on face recognitions and found obstacles */

class person_detector_class
{
private:
  //ros-stuff
  ros::NodeHandle n_;                                           //!< The mandatory ROS nodehandler
  ros::Subscriber sub_face_recognition_;                        //!< Subscriber to cob_people_detection face recognitions
  ros::Publisher pub_all_recognitions_;                         //!< Publisher of all face recognitions
  ros::Publisher pub_all_obstacles_;                            //!< Publisher of all obstacles
  ros::Subscriber sub_map_;                                     //!< Subscriber to the map topic
  ros::Subscriber sub_local_costmap_;                           //!< Subscriber to the local costmap of move_base
  ros::Subscriber sub_obstacles_;                               //!< Subscriber to the published obstacles by move_base
  ros::Subscriber sub_imu_;                                     //!< Subscriber to the robots gyro data
  ros::Subscriber sub_confirmations_;                           //!< Subscriber to the confirmations published by other nodes
  ros::Subscriber sub_amcl_;                                    //!< Subscriber to the robots positions published by amcl
  //transformations
  tf::TransformListener tf_listener_;                           //!< Transformation listener to get transformations between a face recognition and the map
  tf::StampedTransform transform_li_;                           //!< Resuable transformation object for the transformation listener
  tf::TransformBroadcaster tf_human_local_broadcaster_;         //!< Transformation broadcaster to announce transformations between the camera and a detected faces
  tf::Transform transform_br_;                                  //!< Resuable transformation object for the transformation broadcaster
  tf::TransformBroadcaster tf_map_human_broadcaster_;           //!< Transformation broadcaster to announce transformations between the map and the detected faces
  tf::Transform transform_br_map_;                              //!< Reusable transformation object for the transformation broadcaster
  ros::Duration tf_cache_;                                      //!< Storing the information about the length of the tf_listener_ cache

  //markers for rviz
  ros::Publisher pub_human_marker_raw_;                         //!< Publisher of the raw received face detections as cubes in rviz. \sa showAllRecognitions
  visualization_msgs::Marker heads_raw;                        //!< Reusable Marker for the pub_human_marker_raw_. Avoids long initialization. \sa showAllRecognitions
  ros::Publisher pub_human_marker_raw_text_;                    //!< Publisher of the raw received face detections as text in rviz. \sa showAllRecognitions
  visualization_msgs::Marker text_raw_;                         //!< Reusable Marker for the pub_human_marker_raw_text_. Avoids long initialization. \sa showAllRecognitions
  ros::Publisher pub_human_marker_;                             //!< Publisher of all face recognitions stored in all_detections_ as cubes in rviz. \sa showAllRecognitions
  visualization_msgs::Marker heads_;                            //!< Reusable Marker for the pub_human_marker_. Avoids long initialization. \sa showAllRecognitions
  ros::Publisher pub_human_marker_text_;                        //!< Publisher of text to all face recognitions stored in all_detections_ in rviz. \sa showAllRecognitions
  visualization_msgs::Marker heads_text_;                       //!< Reusable Marker for the pub_human_marker_text_. Avoids long initialization. \sa showAllRecognitions
  ros::Publisher pub_obstacle_points_text_;                     //!< Publisher for information to every occupied point in the difference_map_ \sa showAllObstacles_ \sa difference_map_
  visualization_msgs::Marker obstacle_points_text_;             //!< Reusable Marker for the pub_obstacle_points_text_. Avoids long intializiation. \sa showAllObstacles_ \sa pub_obstacle_points_text_
  ros::Publisher pub_obstacle_borders_;                         //!< Publisher for a line connecting all points of an obstacle in rviz. \sa showAllObstacles_
  visualization_msgs::Marker obstacle_boarder_marker_;          //!< Reusable Marker for the pub_obstacle_boarders_. Avoids long initialization. \sa showAllObstacles_ \sa pub_obstacle_boarders_
  ros::Publisher pub_obstacle_cubes_;                           //!< Publisher for a cube representing an the middle of an obstacle in rviz. \sa showAllObstacles_
  visualization_msgs::Marker obstacle_cubes_;                   //!< Reusbale Marker for the pub_obstacle_cubes_. Avoids long initialization. \sa showAllObstacles_ \sa pub_obstacle_cubes_
  ros::Publisher pub_obstacle_info_text_;                       //!< Publisher for text information to each obstacle in rviz. \sa showAllObstacles_
  visualization_msgs::Marker obstacle_info_text_;               //!< Reusable Marker for the pub_obstacle_info_text_. Avoids long intialization. \sa showAllObstacles_ \sa pub_obstacle_info_text_


  //callbacks
  //! Callback for face recognitions of the cob_peoble_detection.
  /*! \param received_detections detection array from cob_peoble_detection*/
  void faceRecognitionCallback_(const cob_people_detection_msgs::DetectionArray received_detections);

  //! Callback for the occupancy map used by the robot.
  /*! \param received_map The received map */
  void mapCallback_(const nav_msgs::OccupancyGrid received_map);

  //! Callback for the localCostmap provided by move_base
  /*! \param received The received occupancy grid */
  void localCostmapCallback_ (const nav_msgs::OccupancyGrid received);

  //! Callback for the occupied points provided by move_base
  /*! \param pcl The received PointCloud */
  void obstaclesCallback_ (const sensor_msgs::PointCloud pcl);

  //! Callback for the gyrometer output.
  /*! \param imu The received imu data */
  void imuCallback_(const sensor_msgs::Imu imu);

  //! Callback for confirmations information about an obstacle
  /*! \param conf The received confirmation information */
  void confirmationCallback_(const person_detector::SpeechConfirmation conf);

  //! Callback for the position of the robot provided by amcl
  /*! \param pose The received pose with covariance */
  void amclCallback_(const geometry_msgs::PoseWithCovarianceStamped pose);

  //detections
  //! Storage for detection arrays provided by cob_people_detection waiting to be processed
  /*! The detections are just saved here until they are processed and matched with transformation information. A warn is sent out if this array becomes too big.
      \sa processDetections */
  std::queue<cob_people_detection_msgs::DetectionArray> detection_temp_storage_;

  //! The array of all current detections with all information.
  /*! This array is frequently updated with new detections and new incoming data and sent out by the publisher.
      \sa processDetection \sa pub_all_recognitions_ */
  person_detector::DetectionObjectArray all_detections_array_;

  //! A counter for the unique IDs assigned to all recognitions.
  /*! It has to incremented every time a new detected obstacle or a new detected face is is created. Obstacles and face recognitions share the same counter.*/
  unsigned int recognition_id_;

  //! Holds a copy of the static map received from the /map topic.
  /*! If a new map is received, the map information is copied into this costmap. After that it will be inflated by 10cm.
      \sa inflateMap_ \sa mapCallback_ */
  costmap_2d::Costmap2D static_map;

  //! This map is frequently updated with occupancy information.
  /*! It stores the information about currently occupied and free points and receives information from the localCostmap of move_base and the published obstacles. It is used to calculate the difference map.
      \sa localCostmapCallback_ \sa obstaclesCallback_ \sa calcDifferenceMap \sa difference_map_ */
  costmap_2d::Costmap2D updated_map;

  //! This map stores information about the distance from which the obstacle has been seen.
  /*! The depthdata of the Kinect is very inaccurate on higher distances. The closest distance of a detection for each point is saved in this map. Please note, that it is stored in decimeter. A value of 10 results in 100cm closest distance. Obstacle seen by the localCostmap are always stored with 100cm distance. This map is used by rateObstacle.
      \sa localCostmapCallback \sa obstacleCallback \sa rateObstacle */
  costmap_2d::Costmap2D updated_dm_;

  //! This map stores information about the number of appearances of an obstacle.
  /*! Sometimes obstacles are just seen a few times because it were walking people or wrong sensor information. This map counts the appearances of each occupied point. The maximum value is 255 according to the limit of unsigned char. If a occupied is marked as FREE_SPACE by the localCostmap, 10 points are substracted each time.
      \sa localCostmapCallback \sa obstaclesCallback_ \sa rateObstacle_ */
  costmap_2d::Costmap2D updated_counter_;

  //! This map is the difference between the updated map and the inflated static map
  /*! Every point which is occupied on the updated map but not occupied in the inflated static map is occupied in this map. This map is the base for the detection of obstacles.
      \sa updatedMap_ \sa static_map_ \sa generateDifferenceMap \sa findObstacles */
  costmap_2d::Costmap2D difference_map_;

  //! This map is used in the process of finding and rematching of obstacles and is copy of difference_map_
  /*! The usage of this map is not really sure. Each run it is a copy of the differenceMap_ and every processed occupied point is marked as FREE_SPACE.
      \todo Check if the dmap_new_ is really necessary. Probably not.*/
  costmap_2d::Costmap2D dmap_new_;

  //! The seen obstacles during a panorama turn are marked in this map.
  /*! This map isn't used yet, but it should help to report which obstacles have been seen during a panorama turn.*/
  costmap_2d::Costmap2D dmap_pano_;

  //! Marks if the maps have been intialized by retrieving the map from the map topic.
  /*! In order to start working it is very important to retrieve the map from the map topic first. The most functions of this node don't work if the maps haven't been initialized yet.
      \todo Add a warn if map_initialized is false */
  bool map_initialized_;

  costmap_2d::Costmap2DPublisher *pub_static_map;                               //!< Publisher for the static map.  \sa static_map_
  costmap_2d::Costmap2DPublisher *pub_updated_map;                              //!< Publisher for the updated map representing the current obstacles \sa updated_map_
  costmap_2d::Costmap2DPublisher *pub_difference_map;                           //!< Publisher for the difference map \sa difference_map_
  costmap_2d::Costmap2DPublisher *pub_dmap_new_;                                //!< Publisher for the difference map used to find obstacles \sa dmap_new_
  costmap_2d::Costmap2DPublisher *pub_dmap_pano_;                               //!< Publisher for the dmap_pano_ \sa dmap_pano_

  //! Storing the rotation velocity provided by the gyrometer
  /*! The latest angular velocity in z-direction (rotation of the robot) is stored here. This is necessary because the occupied points received by the obstacle publisher of move_base are too noisy during a turn.
      \sa obstacleCallback_ */
  double imu_ang_vel_z;

  //! A queue storing all confirmations until they are processed
  std::queue<person_detector::SpeechConfirmation> conf_queue_;

  //! A vector storing the latest 30 amcl poses of the robot
  /*! The poses have to be stored, because the storage of information from localCostmap needs the position of the robot.
      \sa findAmclPose_ */
  std::vector<geometry_msgs::PoseWithCovarianceStamped> amcl_poses_;

  //! All information about all current tracked obstacles are stored in here.
  /*! All information about obstacles are stored in this array. It is frequently updated and later published. Each obstacle entry must have an corresponding entry in all_obs_map_xy_. So the size and the order of the obstacle vector in all_obstacles_ must be the same as in all_obs_map_xy_
      \sa findObstacles_ \sa all_obs_map_xy_ */
  person_detector::ObstacleArray all_obstacles_;

  //! This vector holds the corresponding points of an obstacle in the array of the costmap
  /*! In order to avoid frequent conversion from metric map coordinates to the corresponding points in the array of the costmap, all occupied points of an obstacle are stored in here as well. The size and the order of this vector must be the same as the number of obstacle in all_obs_map_xy_
       \sa ObsMapPoints \sa all_obstacles_*/
  std::vector<person_detector::ObsMapPoints> all_obs_map_xy_;
  //global helperpoint
  geometry_msgs::Point p;                                                     //!< This point is globally used to avoid the creation of temporary ones. Always reset unused attributes!
  geometry_msgs::Point p_map_xy;                                              //!< This point is globally used to avoid the creation of temporary ones. Always reset unused attributes!
  double x_map;                                                               //!< This variable is globally used to avoid the creation of temporary ones.
  double y_map;                                                               //!< This variable is globally used to avoid the creation of temporary ones.

  //functions
  //! This function processes incoming detections to the right coordinate frame, rates and adds them to the global array.
  /*! \return¸Sucess of the processing */
  int processDetections_();

  //! This function takes incoming detections and calculates the distance to known ones.
  /*! \param detection_array The incoming detections
      \return 0 on success
      \todo Function always return true - can it never fail?
      \sa processDetections */
  int classifyDetections_( cob_people_detection_msgs::DetectionArray detection_array );

  //! This function adds a incoming detection to the array of all detections
  /*! \param new_detection The new incoming detection which should be added
      \return 0 on sucess
      \sa updateDetection_ */
  int addNewDetection_(cob_people_detection_msgs::Detection new_detection);

  //! This function updates a known detection with new information,
  /*! \param new_detection The incoming detection delivering information for the update
      \param pos The position in the all_detections_array_ for array access
      \return 0 on sucess
      \sa addNewDetection_*/
  int updateDetection_(cob_people_detection_msgs::Detection new_detection, unsigned int pos);

  //! This function finds the closest known detection to a incoming detection
  /*! \param distances An array of distances
      \param win_id The ID of the closest known detection to each incoming detection. The same ID can appear several times!
      \param win_dist The winning distance for each pair in meter.
      \param detection_array_size The amount of incoming detections
      \return 0 on success
*/
  int findDistanceWinner_(std::vector< std::vector <double> > &distances, std::vector<unsigned int> &win_id, std::vector<double> &win_dist, unsigned int detection_array_size);

  //! Checks if two incoming detections want to assign to the same known detection
  /*! \param distances The array of all distances between a incoming and a known detection
      \param win_id The ID of the closest known detection to an incoming detection
      \param win_dist   The shortest distance between the incoming detection and the known detection specified in win_id
      \param detection_array_size The amount of known incoming detections
      \return 0 on success*/
  int clearDoubleResults_(std::vector< std::vector <double> > &distances, std::vector<unsigned int> &win_id, std::vector<double> &win_dist, unsigned int detection_array_size);

  //! Substracts an hit of a name on every other DetectionObject
  /*! \param label The newly detected name
      \param leave_id The ID the name was newly assigned.
      \return 0 on success */
  int substractHit(std::string label, unsigned int leave_id);

  //! Deletes old face recognitions and detected obstacles
  /*! \param oldness The maximum lifetime and old object can have
      \return 0 on success */
  int garbageCollector_ (ros::Duration oldness);

  //! Prepares and sends visualization of the face recognitions to rviz
  void showAllRecognitions();

  //! Generates the difference map from the static map and the updated map
  /*! \return 0 on sucess
      \sa difference_map_ \sa updated_map */
  int generateDifferenceMap();

  //! Updates known obstacles and finds new obstacles
  /*! \return 0 on success
      \sa all_obstacles_ */
  int findObstacles();

  //! Recursive helper function searching for more occupied points around a specified point
  /*! \param orig_x The starting x position on the costmap
      \param orig_y The starting y position on the costmap
      \param costmap The costmap used for to search. Every found occupied point is going to be marked as FREE_SPACE on this costmap
      \param points Vector storing all found points in metric map coordiantes to update or create an obstacle object
      \param points_map_xy Vector storing all found point in costmap array coordinates
      \return sucess */
  bool searchFurther(unsigned int orig_x, unsigned int orig_y, costmap_2d::Costmap2D* costmap, std::vector<geometry_msgs::Point> *points, std::vector<geometry_msgs::Point> *points_map_xy );

  //! Helper function rating an obstacle on a scale from 0 to 100
  /*! \param obs Pointer to the obstacle that should be rated
      \param map_points Pointer to the corresponding points in the costmap array coordinates
      \return sucess */
  bool rateObstacle_(person_detector::Obstacle *obs, person_detector::ObsMapPoints *map_points);

  //! Prepares and sends visualization of the obstacles to rviz
  void showAllObstacles();

  //! Inflates occupied points on a received static map by 10cm
  int inflateMap();

  //! Updates known obstacles and known face recognitions with incoming confirmation information
  int processConfirmations_();

  //! Finds the best fitting robot position to a specified time
  /*! \param pose The returned pose
      \param stamp The time the pose should match
      \result False if no poses are stored. True if a pose could be found*/
  bool findAmclPose_ (geometry_msgs::PoseWithCovarianceStamped &pose, ros::Time stamp);

public:
  //! Constructor initializing subscriber, publisher and marker
  person_detector_class();
  //! Runs endless and manages the whole detection process
  int run();
};

#endif // PERSON_DETECTOR_H
