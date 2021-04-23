#ifdef ENABLE_ROS

#include "ros_interface.h"

#include <iostream>
#include <string>
#include <sstream>
#include <math.h>

#include <cv_bridge/cv_bridge.h>

#include "svs.h"

// Thresholds for determining when to update the scene graph
const double ros_interface::POS_THRESH = 0.001; // 1 mm
const double ros_interface::ROT_THRESH = 0.017; // approx 1 deg

const std::string ros_interface::RGB_IMAGE_NAME = "rgb_image";
const std::string ros_interface::POINT_CLOUD_NAME = "point_cloud";
const std::string ros_interface::SG_NAME = "scene_graph";
const std::string ros_interface::JOINTS_NAME = "arm_pose";

ros_interface::ros_interface(svs* sp)
    : image_source("none") {
    svs_ptr = sp;
    set_help("Control connections to ROS topics.");

    // Set up the maps needed to track which inputs are enabled/disabled
    // and change this via command line
    update_inputs[RGB_IMAGE_NAME] = false;
    update_inputs[POINT_CLOUD_NAME] = false;
    update_inputs[SG_NAME] = false;
    update_inputs[JOINTS_NAME] = false;

    enable_fxns[RGB_IMAGE_NAME] = std::bind(&ros_interface::subscribe_rgb, this);
    enable_fxns[POINT_CLOUD_NAME] = std::bind(&ros_interface::subscribe_pc, this);
    enable_fxns[SG_NAME] = std::bind(&ros_interface::subscribe_sg, this);
    enable_fxns[JOINTS_NAME] = std::bind(&ros_interface::subscribe_joints, this);

    disable_fxns[RGB_IMAGE_NAME] = std::bind(&ros_interface::unsubscribe_rgb, this);
    disable_fxns[POINT_CLOUD_NAME] = std::bind(&ros_interface::unsubscribe_pc, this);
    disable_fxns[SG_NAME] = std::bind(&ros_interface::unsubscribe_sg, this);
    disable_fxns[JOINTS_NAME] = std::bind(&ros_interface::unsubscribe_joints, this);

    // Create the map of ROS Image publishers so SVS components can publish images
    rgb_pubs = std::map<std::string, ros::Publisher>();
}

ros_interface::~ros_interface() {
    stop_ros();
}

// Static funtion to simply call ros::init, which MUST be called
// before creating the first NodeHandle. The NodeHandle is created
// in the class constructor, hence the need for this to be available
// without a class instance.
void ros_interface::init_ros() {
    int argc = 0;
    char* argv;

    ros::init(argc, &argv, "soar_svs");
}

void ros_interface::start_ros() {
    if (!spinner) {
        spinner = new ros::AsyncSpinner(4);
        spinner->start();
    }
}

void ros_interface::stop_ros() {
    if (spinner) spinner->stop();
    image_source = "none";
}

// Thresholded difference between two vectors. Allows us to not
// update the scene graph minor object movements that are below
// the threshold.
bool ros_interface::t_diff(vec3& p1, vec3& p2) {
    // Euclidean distance
    if (sqrt(pow(p1.x() - p2.x(), 2) +
             pow(p1.y() - p2.y(), 2) +
             pow(p1.z() - p2.z(), 2)) > POS_THRESH) return true;
    return false;
}

// Thresholded difference between two quaterions. Allows us to not
// update the scene graph minor object rotations that are below
// the threshold.
bool ros_interface::t_diff(Eigen::Quaterniond& q1, Eigen::Quaterniond& q2) {
    // Calculating the angle between to quaternions
    double a = 2 * acos(q1.dot(q2));
    if (a > ROT_THRESH) return true;
    return false;
}

// Create an image publisher with a unique string ID for an SVS component
void ros_interface::create_rgb_publisher(std::string pub_ID) {
    std::string topic_name = "svs_topics/"+pub_ID;
    rgb_pubs[pub_ID] = n.advertise<sensor_msgs::Image>(topic_name, 5);
    ROS_INFO("Publishing new svs_topic %s", pub_ID.c_str());
}

void ros_interface::publish_rgb_image(std::string pub_ID, cv::Mat image) {
    sensor_msgs::Image* msg_image = new sensor_msgs::Image();
    msg_image->data = std::vector<uchar>(image.datastart, image.dataend);
    msg_image->encoding = "bgra8";
    msg_image->header = std_msgs::Header();
    msg_image->height = image.rows;
    msg_image->width = image.cols;
    msg_image->is_bigendian = 0;
    msg_image->step = image.cols*image.elemSize();

    sensor_msgs::Image::ConstPtr msg(msg_image);
    rgb_pubs[pub_ID].publish(msg);
}

// Subscribes to an opencv image publisher
void ros_interface::subscribe_rgb() {
    rgb_sub = n.subscribe("/wow_mom_image_topic", 5, &ros_interface::rgb_callback, this);
    image_source = "opencv";
    update_inputs[RGB_IMAGE_NAME] = true;
}

// Unsubscribes to an opencv image publisher
void ros_interface::unsubscribe_rgb() {
    rgb_sub.shutdown();
    update_inputs[RGB_IMAGE_NAME] = false;
    image_source = "none";
    sensor_msgs::Image::ConstPtr empty(new sensor_msgs::Image);
    rgb_callback(empty);
}

// Subscribes to the Fetch's point cloud topic
void ros_interface::subscribe_pc() {
    pc_sub = n.subscribe("head_camera/depth_registered/points", 5, &ros_interface::pc_callback, this);
    image_source = "fetch";
    update_inputs[POINT_CLOUD_NAME] = true;
}

// Unsubscribes from the point cloud topic and updates Soar with
// an empty image
void ros_interface::unsubscribe_pc() {
    pc_sub.shutdown();
    update_inputs[POINT_CLOUD_NAME] = false;
    image_source = "none";
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr empty(new pcl::PointCloud<pcl::PointXYZRGB>);
    pc_callback(empty);
}

// Subscribes to the Gazebo object models
void ros_interface::subscribe_sg() {
    objects_sub = n.subscribe("gazebo/model_states", 5, &ros_interface::objects_callback, this);
    update_inputs[SG_NAME] = true;
}

// Unsubscribes from Gazebo models and updates Soar with an empty scene graph
void ros_interface::unsubscribe_sg() {
    objects_sub.shutdown();
    update_inputs[SG_NAME] = false;
    gazebo_msgs::ModelStates::ConstPtr empty(new gazebo_msgs::ModelStates);
    objects_callback(empty);
}

// Subscribes to the Fetch's joint states
void ros_interface::subscribe_joints() {
    joints_sub = n.subscribe("joint_states", 5, &ros_interface::joints_callback, this);
    update_inputs[JOINTS_NAME] = true;
}

// Unsubscribes joint state
void ros_interface::unsubscribe_joints() {
    joints_sub.shutdown();
    update_inputs[JOINTS_NAME] = false;
    //gazebo_msgs::ModelStates::ConstPtr empty(new gazebo_msgs::ModelStates);
    //objects_callback(empty);
}

// Updates the OpenCV-based 2D RGB percept buffer in the visual sensory memory
void ros_interface::rgb_callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr msg_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGRA8);
    svs_ptr->image_callback(msg_image_ptr->image);
}

// Adds relevant commands to the input list in the main SVS class
// when a new world state is received from Gazebo
void ros_interface::objects_callback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    // First translate the message into a map of object names->xforms
    std::map<std::string, transform3> current_objs;

    for (int i = 0; i <  msg->name.size(); i++) {
        geometry_msgs::Pose pose = msg->pose[i];
        std::string n = msg->name[i];

        vec3 p(pose.position.x,
               pose.position.y,
               pose.position.z);
        Eigen::Quaterniond q(pose.orientation.w,
                             pose.orientation.x,
                             pose.orientation.y,
                             pose.orientation.z);
        // XXX: Is this right? Euler order in exsiting SVS?
        vec3 r = q.toRotationMatrix().eulerAngles(0, 1, 2);

        transform3 t(p, r, vec3(1, 1, 1));
        current_objs.insert(std::pair<std::string, transform3>(n, t));
    }

    // XXX: Eventually want to use the map to directly update the scene graph
    //      instead of going through SGEL. Will require threadsafe scene
    //      graphs.

    // Build up a string of commands in the stringsream
    std::stringstream cmds;
    // But only bother sending the update to SVS if something changed
    bool objs_changed = false;

    // ADD commands for NEW objects that are present in the current msg
    // but are not present in SVS's scene graph
    for (std::map<std::string, transform3>::iterator i = current_objs.begin();
         i != current_objs.end(); i++) {
        if (last_objs.count(i->first) == 0) {
            objs_changed = true;
            std::string n = i->first;
            vec3 cur_pose;
            i->second.position(cur_pose);
            Eigen::Quaterniond rq;
            i->second.rotation(rq);
            vec3 cur_rot = rq.toRotationMatrix().eulerAngles(0, 1, 2);

            // SGEL
            cmds << "add " << n << " world ";
            cmds << "p " << cur_pose.x() << " " << cur_pose.y() << " " << cur_pose.z();
            cmds << " r " << cur_rot.x() << " " << cur_rot.y() << " " << cur_rot.z();
            cmds << std::endl;
        }
    }

    for (std::map<std::string, transform3>::iterator i = last_objs.begin();
         i != last_objs.end(); i++) {
        // DELETE commands for objects that are NOT present in the msg but
        // are still present in SVS's scene graph
        if (current_objs.count(i->first) == 0) {
            objs_changed = true;

            // SGEL
            cmds << "delete " << i->first << " " << std::endl;
            continue;
        }

        // CHANGE commands for objects that are present in both the msg and
        // SVS scene graph but have changed position or rotation
        std::string n = i->first;
        vec3 last_pose;
        i->second.position(last_pose);
        vec3 cur_pose;
        current_objs[n].position(cur_pose);
        Eigen::Quaterniond last_rot;
        i->second.rotation(last_rot);
        Eigen::Quaterniond cur_rot;
        current_objs[n].rotation(cur_rot);

        // Check that at least one of the differences is above the thresholds
        if (t_diff(last_pose, cur_pose) || t_diff(last_rot, cur_rot)) {
            objs_changed = true;

            // SGEL
            cmds << "change " << n;
            if (t_diff(last_pose, cur_pose)) {
                cmds << " p " << cur_pose.x() << " " << cur_pose.y() << " " << cur_pose.z();
            }
            if (t_diff(last_rot, cur_rot)) {
                vec3 rpy = cur_rot.toRotationMatrix().eulerAngles(0, 1, 2);
                cmds << " r " << rpy.x() << " " << rpy.y() << " " << rpy.z();
            }
            cmds << std::endl;
        }
    }

    last_objs = current_objs;
    if (objs_changed) {
        // Send the compiled commands to the SVS input processor
        svs_ptr->add_input(cmds.str());
    }
}

// (Will do something?) when a new arm position is received
void ros_interface::joints_callback(const sensor_msgs::JointState::ConstPtr& msg) {
    //std::cout << "Received joints!" << std::endl;
}

// Updates the images in SVS states when a new point cloud is received
void ros_interface::pc_callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg) {
    svs_ptr->image_callback(msg);

    // TODO: fix pc_callback to convert point clouds into flat 2d images

    // if (!msg->isOrganized()) { return; }  // Can't convert unorganized cloud to cv::Mat

    // // Convert to OpenCV Mat and update OpenCV image as well
    // int img_rows = msg->height;
    // int img_cols = msg->width;

    // cv::Mat depth_map = cv::Mat(img_rows, img_cols, CV_32FC1);
    // cv::Mat flat_image = cv::Mat(img_rows, img_cols, CV_8UC3);
    
    // for(int r=0; r < img_rows; r++) {
    //     for(int c=0; c < img_cols; c++) {
    //         pcl::PointXYZRGB point = msg->at(r, c);
    //         cv::Vec3b point_color = cv::Vec3b(point.b, point.g, point.r);

    //         depth_map.at<float>(r, c) = point.z;
    //         flat_image.at<cv::Vec3b>(r, c) = point_color;
    //     }
    // }

    // svs_ptr->image_callback(flat_image);
}

void ros_interface::proxy_get_children(std::map<std::string, cliproxy*>& c) {
    c["enable"] = new memfunc_proxy<ros_interface>(this, &ros_interface::enable);
    c["enable"]->add_arg("INPUT", "Name of the input to enable.");
    c["disable"] = new memfunc_proxy<ros_interface>(this, &ros_interface::disable);
    c["disable"]->add_arg("INPUT", "Name of the input to disable.");
}

void ros_interface::proxy_use_sub(const std::vector<std::string>& args, std::ostream& os) {
    os << "=========== ROS INPUTS ===========" << std::endl;
    for (std::map<std::string, bool>::iterator i = update_inputs.begin();
         i != update_inputs.end(); i++) {
        os << i->first << ": ";
        if (i->second) os << "enabled";
        else os << "disabled";
        os << std::endl;
    }
    os << "==================================" << std::endl;
}

void ros_interface::enable(const std::vector<std::string>& args, std::ostream& os) {
    if (args.empty()) {
        os << "Specify input to enable: all, ";
        for (std::map<std::string, bool>::iterator i = update_inputs.begin();
             i != update_inputs.end(); i++) {
            if (i != update_inputs.begin()) os << ", ";
            os << i->first;
        }
        os << std::endl;
        return;
    }

    if (args[0] == "all") {
        for (std::map<std::string, bool>::iterator i = update_inputs.begin();
             i != update_inputs.end(); i++) {
            if (!i->second) {
                enable_fxns[i->first]();
            }
        }
        os << "All ROS inputs enabled" << std::endl;
        return;
    }

    if (update_inputs.count(args[0]) == 0) {
        os << "Invalid input name provided" << std::endl;
        return;
    }

    if (!update_inputs[args[0]]) enable_fxns[args[0]]();
    os << args[0] << " ROS input enabled" << std::endl;
}

void ros_interface::disable(const std::vector<std::string>& args, std::ostream& os) {
    if (args.empty()) {
        os << "Specify input to disable: all, ";
        for (std::map<std::string, bool>::iterator i = update_inputs.begin();
             i != update_inputs.end(); i++) {
            if (i != update_inputs.begin()) os << ", ";
            os << i->first;
        }
        os << std::endl;
        return;
    }

    if (args[0] == "all") {
        for (std::map<std::string, bool>::iterator i = update_inputs.begin();
             i != update_inputs.end(); i++) {
            if (i->second) {
                disable_fxns[i->first]();
            }
        }
        os << "All ROS inputs disabled" << std::endl;
        return;
    }

    if (update_inputs.count(args[0]) == 0) {
        os << "Invalid input name provided" << std::endl;
        return;
    }

    if (update_inputs[args[0]]) disable_fxns[args[0]]();
    os << args[0] << " ROS input disabled" << std::endl;
}

#endif
