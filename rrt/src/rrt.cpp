// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well

#include "rrt/rrt.h"
#include "rrt/read_way_point_CSVfile.h"
using namespace std;

// Destructor of the RRT class
RRT::~RRT() {
    // Do something in here, free up used memory, print message, etc.
    ROS_INFO("RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(ros::NodeHandle &nh) : nh_(nh), gen((std::random_device()) ()) {

    // TODO: Load parameters from yaml file, you could add your own parameters to the rrt_params.yaml file
    std::string pose_topic, scan_topic;

    nh_.getParam("pose_topic", pose_topic);
    nh_.getParam("scan_topic", scan_topic);
    nh_.getParam("inflation", inf);
    nh_.getParam("lookahead_dist", lookahead_dist);
    nh_.getParam("step", step);
    nh_.getParam("goal_threshold", goal_threshold);
    nh_.getParam("iteration", iteration);
    nh_.getParam("angle_factor", angle_factor);
    nh_.getParam("nominal_speed", nominal_speed);
    nh_.getParam("angle_speed", angle_speed);
    nh_.getParam("sample_depth", sample_depth);
    nh_.getParam("sample_width", sample_width);
    nh_.getParam("neighbor_dist", neighbor_dist);
    
    vector<vector<float>> waypoint_data_long = read_way_point_CSVfile("/home/siyan/SiyanWang_ws/src/f1_10/rrt/test.csv"); // gtpose
    waypoint_length = waypoint_data_long[0].size();
    vector<float> waypoint_data1;
    vector<float> waypoint_data2;
    vector<float> waypoint_data3;
    for (int i = 0; i < waypoint_length; i+=1) { // 800
        waypoint_data1.push_back(waypoint_data_long[0][i]);
        waypoint_data2.push_back(waypoint_data_long[1][i]);
       waypoint_data3.push_back(waypoint_data_long[2][i]);
    }
    waypoint_data.push_back(waypoint_data1);
    waypoint_data.push_back(waypoint_data2);
    waypoint_data.push_back(waypoint_data3);
    waypoint_length = waypoint_data[0].size();

    // pre-set the visualization objects ////////////////////////////////////////////////////////
    waypoint_pub = nh_.advertise<visualization_msgs::Marker>( "/waypoint_vis", 0 );
    tree_lines_pub = nh_.advertise<visualization_msgs::Marker>( "/tree_lines", 0 );
    tree_nodes_pub = nh_.advertise<visualization_msgs::Marker>( "/tree_nodes", 0 );
    path_line_pub = nh_.advertise<visualization_msgs::Marker>( "/path_lines", 0 );
    waypoint_marker.header.frame_id = "map"; waypoint_marker.ns = "waypoint_vis";
    waypoint_marker.type = visualization_msgs::Marker::SPHERE; waypoint_marker.action = visualization_msgs::Marker::ADD;
    waypoint_marker.scale.x = 0.2; waypoint_marker.scale.y = 0.2; waypoint_marker.scale.z = 0.1;
    waypoint_marker.color.a = 1.0; waypoint_marker.color.r = 0.0; waypoint_marker.color.g = 1.0; waypoint_marker.color.b = 0.0;
    waypoint_marker.id = 0;
    tree_node_marker.header.frame_id = "map"; tree_node_marker.ns = "tree_lines_vis";
    tree_node_marker.type = visualization_msgs::Marker::SPHERE; tree_node_marker.action = visualization_msgs::Marker::ADD;
    tree_node_marker.lifetime = ros::Duration(0.05);
    tree_node_marker.scale.x = 0.1; tree_node_marker.scale.y = 0.1; tree_node_marker.scale.z = 0.1;
    tree_node_marker.color.a = 1.0; tree_node_marker.color.r = 1.0; tree_node_marker.color.g = 0.0; tree_node_marker.color.b = 0.0;
    tree_node_marker.id = 0;
    tree_line_marker.header.frame_id = "map"; tree_line_marker.ns = "tree_nodes_vis";
    tree_line_marker.type = visualization_msgs::Marker::LINE_LIST; tree_line_marker.action = visualization_msgs::Marker::ADD;
    tree_line_marker.lifetime = ros::Duration(0.05);
    tree_line_marker.scale.x = 0.05; tree_line_marker.scale.y = 0.05; tree_line_marker.scale.z = 0.05;
    tree_line_marker.color.a = 1.0; tree_line_marker.color.r = 0.0; tree_line_marker.color.g = 0.0; tree_line_marker.color.b = 1.0;
    tree_line_marker.id = 0;
    path_line_marker.header.frame_id = "map"; path_line_marker.ns = "path_line";
    path_line_marker.type = visualization_msgs::Marker::LINE_STRIP; path_line_marker.action = visualization_msgs::Marker::ADD;
    path_line_marker.lifetime = ros::Duration(0.05);
    path_line_marker.scale.x = 0.15; path_line_marker.scale.y = 0.15; path_line_marker.scale.z = 0.1;
    path_line_marker.color.a = 1.0; path_line_marker.color.r = 0.0; path_line_marker.color.g = 1.0; path_line_marker.color.b = 0.0;
    path_line_marker.id = 0;
    
    // ROS publishers // TODO: create publishers for the the drive topic, and other topics you might need
    drive_pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("drive", 10);
    map_pub = nh_.advertise<nav_msgs::OccupancyGrid>("gridMap", 1, true);

    // ROS subscribers // TODO: create subscribers as you need

     pose_sub = nh_.subscribe(pose_topic, 10, &RRT::pf_callback, this);
    scan_sub_ = nh_.subscribe(scan_topic, 10, &RRT::scan_callback, this);
   // pose_sub = nh_.subscribe("gt_pose", 1, &RRT::pf_callback, this);


    //  create a occupancy grid
    boost::shared_ptr<nav_msgs::MapMetaData const> levine_metadata_ptr;
    nav_msgs::MapMetaData levine_metadata_msg;
    levine_metadata_ptr = ros::topic::waitForMessage<nav_msgs::MapMetaData>("/map_metadata");
    if (levine_metadata_ptr != nullptr) { levine_metadata_msg = *levine_metadata_ptr; }

    boost::shared_ptr<nav_msgs::OccupancyGrid const> levine_map_ptr;
    levine_map_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map");

    gridMap_static.data = levine_map_ptr->data;
    gridMap_static.header.frame_id = "map";
    gridMap_static.header.stamp = ros::Time::now();
    gridMap_static.info.height = levine_metadata_msg.height;
    gridMap_static.info.width = levine_metadata_msg.width;
    gridMap_static.info.resolution = levine_metadata_msg.resolution;
    gridMap_static.info.origin = levine_metadata_msg.origin;

    gridMap_dynamic.header.frame_id = "map";
    gridMap_dynamic.header.stamp = ros::Time::now();
    gridMap_dynamic.info.height = levine_metadata_msg.height;
    gridMap_dynamic.info.width = levine_metadata_msg.width;
    gridMap_dynamic.info.resolution = levine_metadata_msg.resolution;
    gridMap_dynamic.info.origin = levine_metadata_msg.origin;
    gridMap_dynamic.data.assign(gridMap_dynamic.info.height * gridMap_dynamic.info.width, 0);

    gridMap_final.header.frame_id = "map";
    gridMap_final.header.stamp = ros::Time::now();
    gridMap_final.info.height = levine_metadata_msg.height;
    gridMap_final.info.width = levine_metadata_msg.width;
    gridMap_final.info.resolution = levine_metadata_msg.resolution;
    gridMap_final.info.origin = levine_metadata_msg.origin;
    gridMap_final.data.assign(gridMap_final.info.height * gridMap_final.info.width, 0);

    ROS_INFO("Created new RRT Object.");
}

void RRT::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
    /*
     * The scan callback, update your occupancy grid here
     * Args:
     * scan_msg (*LaserScan): pointer to the incoming scan message
     *
     */
    gridMap_dynamic.data.assign(gridMap_dynamic.info.height * gridMap_dynamic.info.width, 0);

    std::vector<float> ranges = scan_msg->ranges;
    // the following is for dynamic layer
    for (int i = 180; i < 901; i++) { // 180 degrees in front of the car
        float range = ranges[i];
        if (std::isnan(range) || std::isinf(range)) continue; // remove nan and inf
        float angles = scan_msg->angle_min + scan_msg->angle_increment * (float) i;
        float x = range * cos((float) angles), y = range * sin((float) angles); // in laser frame

        // now transform   transform(x,y)
        int col = get_col(transform(x, y).point.x);
        int row = get_row(transform(x, y).point.y);

        //inflation
        for (int j = -inf; j <= inf; j++) {
            for (int k = -inf; k <= inf; k++) {
                int col_inf = col - k;
                int row_inf = row - j; // inf: inflation
                int index_inf = row_col_to_index(row_inf, col_inf);// 1D index
                gridMap_dynamic.data[index_inf] = 100;
            }
        }
    }

    // find out the union set between dynamic and static layer
    for (int k = 1; k <= gridMap_dynamic.info.width * gridMap_dynamic.info.height; k++) {
        gridMap_final.data[k] = (gridMap_static.data[k] || gridMap_dynamic.data[k] != 0) ? 100 : 0; // mark obstacles
    }

    map_pub.publish(gridMap_final); // publish the final grid map
}


void RRT::pf_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) { //geometry_msgs::PoseStamped::ConstPtr &pose_msg
//void RRT::pf_callback(const geometry_msgs::PoseStamped::ConstPtr &odom_msg) { //geometry_msgs::PoseStamped::ConstPtr &pose_msg
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    // Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
    // Returns:

     double currentX = odom_msg->pose.pose.position.x; //vehicle pose X
     double currentY = odom_msg->pose.pose.position.y; //vehicle pose Y
     float currentTheta = convert_to_Theta(odom_msg->pose.pose.orientation);
//    float currentX = odom_msg->pose.position.x;
//    float currentY = odom_msg->pose.position.y;
//    float currentTheta = convert_to_Theta(odom_msg->pose.orientation);

    float csv_offset_x = 0;
    float csv_offset_y = 0;

    // find the closest waypoint
    float waypoint_x = 0;
    float waypoint_y=  0;
    float distance_min = 10000;
    int ind_min = 0;
    for(int i = 0; i < waypoint_length; i += 1){
        double distance = sqrt( pow(currentX - (waypoint_data[0][i] - csv_offset_x), 2) + pow(currentY - (waypoint_data[1][i] - csv_offset_y), 2) );
        rot_waypoint_x = (waypoint_data[0][i] - currentX) * cos(-currentTheta) - (waypoint_data[1][i] - currentY) * sin(-currentTheta);
        rot_waypoint_y = (waypoint_data[0][i] - currentX) * sin(-currentTheta) + (waypoint_data[1][i] - currentY) * cos(-currentTheta);
        if (distance_min > distance && distance >= lookahead_dist && rot_waypoint_x > 0) {
            distance_min = distance;
            ind_min = i;
        }
    }
    waypoint_x = waypoint_data[0][ind_min];
    waypoint_y = waypoint_data[1][ind_min];
    // cout << "1" << endl;
    // prevent waypoint from collision
    if (gridMap_dynamic.data[row_col_to_index(get_row(waypoint_y), get_col(waypoint_x))] == 100) {
        if (ind_min < waypoint_length-1) {
            waypoint_x = waypoint_data[0][ind_min+1];
            waypoint_y = waypoint_data[1][ind_min+1];
        } else {
            waypoint_x = waypoint_data[0][0];
            waypoint_y = waypoint_data[1][0];
        }
    }

    // waypoint visualization
    waypoint_marker.pose.position.x = waypoint_x;
    waypoint_marker.pose.position.y = waypoint_y;
    waypoint_marker.id += 1;
    waypoint_marker.header.stamp = ros::Time();
    waypoint_marker.lifetime = ros::Duration(0.3);
    waypoint_pub.publish(waypoint_marker);

    // RRT main loop //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    std::vector<Node> tree;
    std::vector<Node> path;
    Node root;
    root.x = currentX;
    root.y = currentY;
    root.cost = 0;
    root.is_root = true;
    tree.push_back(root);
    // cout << "2" << endl;
    for(int i = 0; i < iteration; i++){
        std::vector<double> sampled = sample(currentX, currentY, currentTheta);
        nearest_node_index = nearest(tree, sampled);
        Node new_node = steer(tree[nearest_node_index], sampled);
        // float new_node_laser_x = (new_node.x - currentX) * cos(-currentTheta) - (new_node.x - currentY) * sin(-currentTheta);
        // float nearest_node_laser_x = (tree[nearest_node_index].x - currentX) * cos(-currentTheta) - (tree[nearest_node_index].x - currentY) * sin(-currentTheta);
        // if (new_node_laser_x < nearest_node_laser_x) {
        //     continue;
        // }
        // check collision
        bool collision = check_collision(tree[nearest_node_index], new_node);
        if(collision) {
            continue;
        }else{
            new_node.cost = cost(tree, new_node);
            vector<int> neighborhood = near(tree, new_node);
            // cout << neighborhood.size() << endl;
            // Connect along a minimum-cost path
            for (int i = 0; i < neighborhood.size(); i++) {
                float new_cost = tree[neighborhood[i]].cost + line_cost(tree[neighborhood[i]], new_node);
                if (new_node.cost > new_cost) {
                    new_node.cost = new_cost;
                    new_node.parent = neighborhood[i];
                }
            }
            // Rewire the tree
            for (int i = 0; i < neighborhood.size(); i++) {
                float neighbor_cost = new_node.cost + line_cost(tree[neighborhood[i]], new_node);
                if (tree[neighborhood[i]].cost > neighbor_cost) {
                    tree[neighborhood[i]].cost = neighbor_cost;
                    tree[neighborhood[i]].parent = tree.size(); // position of the new_node
                }
            }
            tree.push_back(new_node);
        }
        // tree line visualization
        tree_line_marker.points.clear();
        geometry_msgs::Point p1;
        p1.x = tree[nearest_node_index].x;
        p1.y = tree[nearest_node_index].y;
        tree_line_marker.points.push_back(p1);
        geometry_msgs::Point p2;
        p2.x = new_node.x;
        p2.y = new_node.y;
        tree_line_marker.points.push_back(p2);
        tree_line_marker.id += 1;
        tree_lines_pub.publish(tree_line_marker);
        // check if goal is reached
        if(is_goal(new_node, waypoint_x, waypoint_y)){
            path = find_path(tree, new_node);
            break;
        }
    }
    // cout << "3" << endl;
    // tree node visualization
    for (int i = 0; i < tree.size(); i++) {
        tree_node_marker.pose.position.x = tree[i].x;
        tree_node_marker.pose.position.y = tree[i].y;
        tree_node_marker.id += 1;
        tree_nodes_pub.publish(tree_node_marker);
    }
    
    // if there is no path, use the previous found path
    if (path.size() == 0) {
        // cout << "no path" << endl;
        path = old_path;
    } else {
        old_path = path;
    }
    // path line visualization
    geometry_msgs::Point p3;
    path_line_marker.points.clear();
    for (int i = 0; i < path.size(); i++) {
        p3.x = path[i].x;
        p3.y = path[i].y;
        path_line_marker.points.push_back(p3);
    }
    path_line_pub.publish(path_line_marker);
    // cout << "4" << endl;

    float short_term_goal = 0.5;
    if (path.size() > 0) {
        // find the closest point in pass
        distance_min = 10000;
        ind_min = 0;
        for(int i = 0; i < path.size(); i += 1){
            double distance = sqrt( pow(currentX - (path[i].x - csv_offset_x), 2) + pow(currentY - (path[i].y - csv_offset_y), 2) );
            float rot_passpoint_x = (path[i].x - currentX) * cos(-currentTheta) - (path[i].y - currentY) * sin(-currentTheta);
            float rot_passpoint_y = (path[i].x - currentX) * sin(-currentTheta) + (path[i].y - currentY) * cos(-currentTheta);
            if (distance_min > distance && rot_passpoint_x > 0 && distance >= short_term_goal) { //&& distance >= short_term_goal
                distance_min = distance;
                ind_min = i;
            }
        }
        float passpoint_x = path[ind_min].x;
        float passpoint_y = path[ind_min].y;
        float rot_passpoint_x = (passpoint_x - currentX) * cos(-currentTheta) - (passpoint_y - currentY) * sin(-currentTheta);
        float rot_passpoint_y = (passpoint_x - currentX) * sin(-currentTheta) + (passpoint_y - currentY) * cos(-currentTheta);  
        steering_angle = angle_factor * (2 * rot_passpoint_y) / (pow(rot_passpoint_x, 2) + pow(rot_passpoint_y, 2));
        
        ackermann_msgs::AckermannDriveStamped drive_msg;
        if(steering_angle < -0.4189){
            steering_angle = -0.4189;
        }
        if(steering_angle > 0.4189){
            steering_angle = 0.4189;
        }
        drive_msg.drive.steering_angle = steering_angle; //Sets steering angle
        drive_msg.header.stamp = ros::Time::now();
        drive_msg.header.frame_id = "laser";
        drive_msg.drive.speed = nominal_speed - (nominal_speed - angle_speed) * fabs(steering_angle) / 0.4189;
        drive_pub.publish(drive_msg); //Sets velocity based on steering angle conditions
    }

}

geometry_msgs::PointStamped RRT::transform(double x, double y) {
    /*
     * transformation between /laser and /map frames
     * Input: x anf y of laser scan in laser frame
     * Return: transformed point in map frame
     */

    geometry_msgs::PointStamped before_transform;
    before_transform.header.frame_id = "/laser";
    before_transform.point.x = x;
    before_transform.point.y = y;

    geometry_msgs::PointStamped after_transform;
    after_transform.header.frame_id = "/map";
    try {
        listener.transformPoint("/map", before_transform, after_transform);

    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    return after_transform; // return transformed point
}

int RRT::row_col_to_index(int row, int col) {
    return row * gridMap_dynamic.info.width + col; // 1D index in grid map
}

int RRT::get_col(double x) {
    int col = static_cast<int>((x - gridMap_dynamic.info.origin.position.x) / gridMap_dynamic.info.resolution);
    return col;// column in grid map
}

int RRT::get_row(double y) {
    int row = static_cast<int>((y - gridMap_dynamic.info.origin.position.y) / gridMap_dynamic.info.resolution); // cell
    return row; // row in grid map
}

double RRT::convert_to_Theta(geometry_msgs::Quaternion msg){
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    //theta is just the yaw angle
    return yaw;
}

std::vector<double> RRT::sample(float currentX, float currentY, float currentTheta) {
    // This method returns a sampled point from the free space
    // You should restrict so that it only samples a small region
    // of interest around the car's current position
    // Args:
    // Returns:
    //     sampled_point (std::vector<double>): the sampled point in free space
    
    std::uniform_real_distribution<double> x_dist{0.2, sample_depth}; // need tuning
    std::uniform_real_distribution<double> y_dist{-sample_width, sample_width}; // need tuning
    std::vector<double> sampled_point;
    bool flag = true;
    while (flag) {
        double x = x_dist(gen); // x and y range in header file could be adjusted
        double y = y_dist(gen); //
        // transform from laser frame to map frame
        // float rot_x = (x - currentX) * cos(-currentTheta) - (y - currentY) * sin(-currentTheta);
        // float rot_y = (x - currentX) * cos(-currentTheta) - (y - currentY) * sin(-currentTheta);
        int col = get_col(transform(x, y).point.x);
        int row = get_row(transform(x, y).point.y);
        // int col = get_col(rot_x);
        // int row = get_row(rot_y);
        int index = row_col_to_index(row, col); // get index in grid map
        
        if (gridMap_final.data[index] == 0) { // check free space
            sampled_point.push_back(transform(x, y).point.x);
            sampled_point.push_back(transform(x, y).point.y);
            // sampled_point.push_back(rot_x);
            // sampled_point.push_back(rot_x);
            flag = false;
        }
    }

    return sampled_point;
}

int RRT::nearest(std::vector<Node> &tree, std::vector<double> &sampled_point) {
    // This method returns the nearest node on the tree to the sampled point
    // Args:
    //     tree (std::vector<Node>): the current RRT tree
    //     sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //     nearest_node (int): index of nearest node on the tree

    int nearest_node = 0;
    double current_dist = 0;
    double min_dist = FLT_MAX;

    for (int i = 0; i < tree.size(); i++) {
        current_dist = sqrt(pow(tree[i].x - sampled_point[0], 2) + pow(tree[i].y - sampled_point[1], 2));
        if (current_dist < min_dist) {
            nearest_node = i;
            min_dist = current_dist;
        }
    }

    return nearest_node;
}

Node RRT::steer(Node &nearest_node, std::vector<double> &sampled_point) {
    // The function steer:(x,y)->z returns a point such that z is “closer”
    // to y than x is. The point z returned by the function steer will be
    // such that z minimizes ||z−y|| while at the same time maintaining
    //||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0

    // basically, expand the tree towards the sample point (within a max dist)

    // Args:
    //    nearest_node (Node): nearest node on the tree to the sampled point
    //    sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //    new_node (Node): new node created from steering

    Node new_node;
    double dist = sqrt(pow(nearest_node.x - sampled_point[0], 2) + pow(nearest_node.y - sampled_point[1], 2));
    if (dist < step) {
        new_node.x = sampled_point[0];
        new_node.y = sampled_point[1];
        new_node.is_root = false;
        new_node.parent = nearest_node_index;
    } else {
        // expand the tree towards the sample point
        new_node.x = (step / dist) * (sampled_point[0] - nearest_node.x) + nearest_node.x; //
        new_node.y = (step / dist) * (sampled_point[1] - nearest_node.y) + nearest_node.y;
        new_node.is_root = false;
        new_node.parent = nearest_node_index;
    }

    return new_node;
}

bool RRT::check_collision(Node &nearest_node, Node &new_node) {
    // This method returns a boolean indicating if the path between the
    // nearest node and the new node created from steering is collision free
    // Args:
    //    nearest_node (Node): nearest node on the tree to the sampled point
    //    new_node (Node): new node created from steering
    // Returns:
    //    collision (bool): true if in collision, false otherwise

    bool collision = false;
    int iter = 20; // this could be adjusted
    for (int i = 0; i < iter; i++) {
        float x = (float)(i / iter) * (new_node.x - nearest_node.x) + nearest_node.x;
        float y = (float)(i / iter) * (new_node.y - nearest_node.y) + nearest_node.y;

        int col = get_col(x);
        int row = get_row(y);

        int index = row_col_to_index(row, col); // get index in grid map
        if (gridMap_final.data[index] == 100) {
            collision = true;
            break;
        }
    }

    return collision;
}

bool RRT::is_goal(Node &latest_added_node, double goal_x, double goal_y) {
    // This method checks if the latest node added to the tree is close
    // enough (defined by goal_threshold) to the goal so we can terminate
    // the search and find a path
    // Args:
    //   latest_added_node (Node): latest addition to the tree
    //   goal_x (double): x coordinate of the current goal
    //   goal_y (double): y coordinate of the current goal
    // Returns:
    //   close_enough (bool): true if node close enough to the goal
    bool close_enough = false;
    double dist = sqrt(pow(latest_added_node.x - goal_x, 2) + pow(latest_added_node.y - goal_y, 2));
    close_enough = dist < goal_threshold; // ....
    return close_enough;
}

std::vector<Node> RRT::find_path(std::vector<Node> &tree, Node &node_in) {
    // This method traverses the tree from the node that has been determined
    // as goal
    // Args:
    //   latest_added_node (Node): latest addition to the tree that has been
    //      determined to be close enough to the goal
    // Returns:
    //   path (std::vector<Node>): the vector that represents the order of
    //      of the nodes traversed as the found path

    std::vector<Node> found_path;
    Node latest_added_node = node_in;
    while (!latest_added_node.is_root) {
        found_path.push_back(latest_added_node);
        latest_added_node = tree[latest_added_node.parent];
    }
    return found_path;
}

//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------
// RRT* methods
double RRT::cost(std::vector<Node> &tree, Node &node) {
    // This method returns the cost associated with a node
    // Args:
    //    tree (std::vector<Node>): the current tree
    //    node (Node): the node the cost is calculated for
    // Returns:
    //    cost (double): the cost value associated with the node
    double cost = tree[node.parent].cost +  sqrt( pow(node.x - tree[node.parent].x, 2) + pow(node.y - tree[node.parent].y, 2) );
    return cost;
}

double RRT::line_cost(Node &n1, Node &n2) {
    // This method returns the cost of the straight line path between two nodes
    // Args:
    //    n1 (Node): the Node at one end of the path
    //    n2 (Node): the Node at the other end of the path
    // Returns:
    //    cost (double): the cost value associated with the path

    double cost = sqrt( pow(n1.x - n2.x, 2) + pow(n1.y - n2.y, 2) );

    return cost;
}

std::vector<int> RRT::near(std::vector<Node> &tree, Node &node) {
    // This method returns the set of Nodes in the neighborhood of a
    // node.
    // Args:
    //   tree (std::vector<Node>): the current tree
    //   node (Node): the node to find the neighborhood for
    // Returns:
    //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood

    std::vector<int> neighborhood;
    for (int i = 0; i < tree.size(); i++) {
        float current_dist = sqrt(pow(tree[i].x - node.x, 2) + pow(tree[i].y - node.y, 2));
        if (current_dist < neighbor_dist) {
            neighborhood.push_back(i);
        }
    }
    return neighborhood;
}
