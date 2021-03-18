#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

#ifdef ENABLE_ROS

#include <functional>
#include <map>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/opencv.hpp>

#include "gazebo_msgs/ModelStates.h"
#include "sensor_msgs/JointState.h"

#include "mat.h"
#include "cliproxy.h"

class svs;

/*
 * ros_interface class
 *
 * Provides the necessary boilerplate to make SVS into a ROS
 * node, such as the NodeHandle, init functionality, and subscribers.
 * Includes callback functions that take ROS input and put point
 * cloud data into the image holders and update the scene graph from
 * Gazebo objects.
 *
 * CLI USAGE:
 *
 * svs ros - Prints all inputs and whether they're enabled or disabled
 * svs ros.enable <NAME> - Enables input with name <NAME>
 * svs ros.disable <NAME> - Disables input with name <NAME>
 *
 * <NAME> = "all" (enables/disables all inputs) OR one of inputs listed
 *          by the svs ros command
 */

class ros_interface : public cliproxy {
public:
    ros_interface(svs* sp);
    ~ros_interface();
    static void init_ros();
    void start_ros();
    void stop_ros();

    std::string get_image_source() { return image_source; }

private:
    static const double POS_THRESH;
    static const double ROT_THRESH;
    bool t_diff(vec3& p1, vec3& p2);
    bool t_diff(Eigen::Quaterniond& q1, Eigen::Quaterniond& q2);

    static const std::string RGB_IMAGE_NAME;
    static const std::string POINT_CLOUD_NAME;
    static const std::string SG_NAME;
    static const std::string JOINTS_NAME;

    void create_rgb_publisher(std::string pub_ID);
    void publish_rgb_image(std::string pub_ID, cv::Mat image);
    
    void subscribe_rgb();
    void unsubscribe_rgb();
    void subscribe_pc();
    void unsubscribe_pc();
    void subscribe_sg();
    void unsubscribe_sg();
    void subscribe_joints();
    void unsubscribe_joints();
    void rgb_callback(const sensor_msgs::Image::ConstPtr& msg);
    void objects_callback(const gazebo_msgs::ModelStates::ConstPtr& msg);
    void joints_callback(const sensor_msgs::JointState::ConstPtr & msg);
    void pc_callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg);

    void proxy_get_children(std::map<std::string, cliproxy*>& c);
    void proxy_use_sub(const std::vector<std::string>& args, std::ostream& os);
    void enable(const std::vector<std::string>& args, std::ostream& os);
    void disable(const std::vector<std::string>& args, std::ostream& os);

    ros::NodeHandle n;
    std::map<std::string, bool> update_inputs;
    std::map<std::string, std::function<void()> > enable_fxns;
    std::map<std::string, std::function<void()> > disable_fxns;
    ros::Subscriber rgb_sub;
    ros::Subscriber objects_sub;
    ros::Subscriber joints_sub;
    ros::Subscriber pc_sub;
    std::string image_source;
    ros::AsyncSpinner* spinner;
    std::map<std::string, ros::Publisher> rgb_pubs;
    svs* svs_ptr;
    std::map<std::string, transform3> last_objs;
};

#endif
#endif
