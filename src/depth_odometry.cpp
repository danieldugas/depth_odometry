#include <glog/logging.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>

#include "depth_odometry/iterative_closest_surface.h"

namespace depth_odometry {

class DepthOdomTracker {

  public:
    explicit DepthOdomTracker(ros::NodeHandle& n) : nh_(n) {
      // Topic names.
      const std::string kDepthTopic = "/camera/depth/points";

      // Publishers and subscribers.
      depth_sub_ = nh_.subscribe(kDepthTopic, 100, &DepthOdomTracker::depthCallback, this);
    }
    ~DepthOdomTracker() {}

  protected:
    /// \brief receives joystick messages
    void depthCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
      // Frame constants.
      const std::string kBaseFrameName = "world";
      const std::string kOdomFrameName = "odom";

      if ( is_called_for_the_first_time_ ) {
        is_called_for_the_first_time_ = false;
        // initialize odom estimate.TODO
        // ...

        // initialize prev_cloud_ TODO
        // ...
        return;
      }

//       tf:Transform prev_pose_to_new_pose;

      // Publish tf.
//       tf::Transform transform;
//       transform.setOrigin( tf::Vector3(current_x_estimate_, current_y_estimate_, 0.0) );
//       tf::Quaternion quaternion;
//       quaternion.setRPY(0., 0., current_theta_estimate_);
//       transform.setRotation(quaternion);
//       br_.sendTransform(
//           tf::StampedTransform(transform, msg->header.stamp, kBaseFrameName, kOdomFrameName)
//       );

      // Update prev_cloud for next loop. TODO
    }

  private:
    ros::NodeHandle& nh_;
    ros::Subscriber depth_sub_;
    tf::TransformBroadcaster br_;
    bool is_called_for_the_first_time_ = true;
    /// \brief simple odometry estimation using
//     tf::Transform odom_estimate_; 

}; // class DepthOdomTracker

} // namespace depth_odometry

using namespace depth_odometry;

int main(int argc, char **argv) {

  ros::init(argc, argv, "depth_odometry");
  ros::NodeHandle n;
  DepthOdomTracker depth_odom_tracker(n);

  try {
    ros::spin();
  }
  catch (const std::exception& e) {
    ROS_ERROR_STREAM("Exception: " << e.what());
    return 1;
  }
  catch (...) {
    ROS_ERROR_STREAM("Unknown Exception.");
    return 1;
  }

  return 0;
}

