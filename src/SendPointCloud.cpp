// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>
// #include <grpc++/grpc++.h>
// #include "RatSim.grpc.pb.h"
// #include <mutex>

// class PointCloudProcessor {
// private:
//     ros::NodeHandle nh_;
//     ros::Subscriber static_cloud_sub_;
//     ros::Subscriber dynamic_cloud_sub_;
//     ros::Timer timer_;
//     std::unique_ptr<RatSim::PointCloudService::Stub> cloud_stub_;
    
//     // Mutex for thread safety
//     std::mutex static_cloud_mutex_;
//     std::mutex dynamic_cloud_mutex_;
    
//     // Store latest point clouds
//     sensor_msgs::PointCloud2::ConstPtr latest_static_cloud_;
//     sensor_msgs::PointCloud2::ConstPtr latest_dynamic_cloud_;
    
//     // ROS Parameters
//     std::string server_address_;
//     std::string static_cloud_topic_;
//     std::string dynamic_cloud_topic_;
//     double publish_rate_;
    
//     // Colors (RGBA packed into uint32)
//     static constexpr uint32_t STATIC_COLOR = 0xFF00FF00;  // Green
//     static constexpr uint32_t DYNAMIC_COLOR = 0xFFFF0000;  // Red
    
//     void sendCombinedPointClouds() {
//         try {
//             RatSim::PointCloudWithColor cloud_msg;
            
//             // Process static cloud if available
//             {
//                 std::lock_guard<std::mutex> lock(static_cloud_mutex_);
//                 if (latest_static_cloud_) {
//                     pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
//                     pcl::fromROSMsg(*latest_static_cloud_, pcl_cloud);
                    
//                     for (const auto& point : pcl_cloud.points) {
//                         auto* point_with_color = cloud_msg.add_data();
//                         auto* proto_point = point_with_color->mutable_point();
//                         proto_point->set_x(point.x);
//                         proto_point->set_y(point.y);
//                         proto_point->set_z(point.z);
//                         point_with_color->set_color(STATIC_COLOR);
//                     }
//                 }
//             }
            
//             // Process dynamic cloud if available
//             {
//                 std::lock_guard<std::mutex> lock(dynamic_cloud_mutex_);
//                 if (latest_dynamic_cloud_) {
//                     pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
//                     pcl::fromROSMsg(*latest_dynamic_cloud_, pcl_cloud);
                    
//                     for (const auto& point : pcl_cloud.points) {
//                         auto* point_with_color = cloud_msg.add_data();
//                         auto* proto_point = point_with_color->mutable_point();
//                         proto_point->set_x(point.x);
//                         proto_point->set_y(point.y);
//                         proto_point->set_z(point.z);
//                         point_with_color->set_color(DYNAMIC_COLOR);
//                     }
//                 }
//             }
            
//             // Only send if we have points
//             if (cloud_msg.data_size() > 0) {
//                 // Send to server
//                 grpc::ClientContext context;
//                 RatSim::Status response;
//                 grpc::Status status = cloud_stub_->SendPointCloudWithColor(&context, cloud_msg, &response);
                
//                 if (status.ok() && response.status()) {
//                     ROS_DEBUG("Successfully sent combined point cloud (%d points)", 
//                              cloud_msg.data_size());
//                 } else {
//                     ROS_ERROR("Failed to send combined point cloud: %s",
//                              status.ok() ? "Server rejected data" : 
//                              status.error_message().c_str());
//                 }
//             }
            
//         } catch (const std::exception& e) {
//             ROS_ERROR("Error processing combined point clouds: %s", e.what());
//         }
//     }
    
//     void staticCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
//         std::lock_guard<std::mutex> lock(static_cloud_mutex_);
//         latest_static_cloud_ = msg;
//     }
    
//     void dynamicCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
//         std::lock_guard<std::mutex> lock(dynamic_cloud_mutex_);
//         latest_dynamic_cloud_ = msg;
//     }
    
//     void timerCallback(const ros::TimerEvent&) {
//         sendCombinedPointClouds();
//     }

// public:
//     PointCloudProcessor() : nh_("~") {
//         // Load parameters
//         nh_.param<std::string>("server_address", server_address_, "localhost:50051");
//         nh_.param<std::string>("static_cloud_topic", static_cloud_topic_, 
//                               "/hash_fusion/static_clouds");
//         nh_.param<std::string>("dynamic_cloud_topic", dynamic_cloud_topic_, 
//                               "/hash_fusion/dynamic_clouds");
//         nh_.param<double>("publish_rate", publish_rate_, 10.0);  // 10 Hz default
        
//         // Setup gRPC channel and stub
//         auto channel = grpc::CreateChannel(server_address_, 
//                                          grpc::InsecureChannelCredentials());
//         cloud_stub_ = RatSim::PointCloudService::NewStub(channel);
        
//         // Setup subscribers
//         static_cloud_sub_ = nh_.subscribe(static_cloud_topic_, 5, 
//                                         &PointCloudProcessor::staticCloudCallback, 
//                                         this);
//         dynamic_cloud_sub_ = nh_.subscribe(dynamic_cloud_topic_, 5, 
//                                          &PointCloudProcessor::dynamicCloudCallback, 
//                                          this);
        
//         // Setup timer for fixed rate processing
//         timer_ = nh_.createTimer(ros::Duration(1.0/publish_rate_), 
//                                &PointCloudProcessor::timerCallback, 
//                                this);
                                
//         ROS_INFO("Point cloud processor initialized:");
//         ROS_INFO(" - Server address: %s", server_address_.c_str());
//         ROS_INFO(" - Static cloud topic: %s", static_cloud_topic_.c_str());
//         ROS_INFO(" - Dynamic cloud topic: %s", dynamic_cloud_topic_.c_str());
//         ROS_INFO(" - Publish rate: %.1f Hz", publish_rate_);
//     }
// };

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "point_cloud_processor");
//     PointCloudProcessor processor;
//     ros::spin();
//     return 0;
// }

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <grpc++/grpc++.h>
#include "RatSim.grpc.pb.h"
#include <mutex>

class PointCloudProcessor {
private:
    ros::NodeHandle nh_;
    ros::Subscriber static_cloud_sub_;
    ros::Subscriber dynamic_cloud_sub_;
    ros::Subscriber lidar_odom_sub_;
    ros::Timer timer_;
    std::unique_ptr<RatSim::PointCloudService::Stub> cloud_stub_;
    
    // Mutex for thread safety
    std::mutex static_cloud_mutex_;
    std::mutex dynamic_cloud_mutex_;
    std::mutex odom_mutex_;
    
    // Store latest point clouds and odometry
    sensor_msgs::PointCloud2::ConstPtr latest_static_cloud_;
    sensor_msgs::PointCloud2::ConstPtr latest_dynamic_cloud_;
    nav_msgs::Odometry::ConstPtr latest_odom_;
    
    // ROS Parameters
    std::string server_address_;
    std::string static_cloud_topic_;
    std::string dynamic_cloud_topic_;
    std::string lidar_odom_topic_;
    double publish_rate_;
    double height_threshold_;
    
    // Colors (RGBA packed into uint32)
    static constexpr uint32_t STATIC_COLOR = 0xFF00FF00;  // Green
    static constexpr uint32_t DYNAMIC_COLOR = 0xFFFF0000;  // Red
    static constexpr uint32_t LOW_HEIGHT_COLOR = 0xFF00FF00;  // Green for low height points
    
    double getLidarHeight() {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        return latest_odom_ ? latest_odom_->pose.pose.position.z : 0.0;
    }
    
    void sendCombinedPointClouds() {
        try {
            RatSim::PointCloudWithColor cloud_msg;
            double lidar_height = getLidarHeight();
            
            // Process static cloud if available
            {
                std::lock_guard<std::mutex> lock(static_cloud_mutex_);
                if (latest_static_cloud_) {
                    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
                    pcl::fromROSMsg(*latest_static_cloud_, pcl_cloud);
                    
                    for (const auto& point : pcl_cloud.points) {
                        auto* point_with_color = cloud_msg.add_data();
                        auto* proto_point = point_with_color->mutable_point();
                        proto_point->set_x(point.x);
                        proto_point->set_y(point.y);
                        proto_point->set_z(point.z);
                        
                        // Color based on relative height to LIDAR
                        point_with_color->set_color(
                            lidar_height - point.z > height_threshold_ ? 
                            LOW_HEIGHT_COLOR : STATIC_COLOR);
                    }
                }
            }
            
            // Process dynamic cloud if available
            {
                std::lock_guard<std::mutex> lock(dynamic_cloud_mutex_);
                if (latest_dynamic_cloud_) {
                    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
                    pcl::fromROSMsg(*latest_dynamic_cloud_, pcl_cloud);
                    
                    for (const auto& point : pcl_cloud.points) {
                        auto* point_with_color = cloud_msg.add_data();
                        auto* proto_point = point_with_color->mutable_point();
                        proto_point->set_x(point.x);
                        proto_point->set_y(point.y);
                        proto_point->set_z(point.z);
                        
                        // Color based on relative height to LIDAR
                        point_with_color->set_color(
                            lidar_height - point.z > height_threshold_ ? 
                            LOW_HEIGHT_COLOR : DYNAMIC_COLOR);
                    }
                }
            }
            
            // Only send if we have points
            if (cloud_msg.data_size() > 0) {
                grpc::ClientContext context;
                RatSim::Status response;
                grpc::Status status = cloud_stub_->SendPointCloudWithColor(&context, cloud_msg, &response);
                
                if (status.ok() && response.status()) {
                    ROS_DEBUG("Successfully sent combined point cloud (%d points)", 
                             cloud_msg.data_size());
                } else {
                    ROS_ERROR("Failed to send combined point cloud: %s",
                             status.ok() ? "Server rejected data" : 
                             status.error_message().c_str());
                }
            }
            
        } catch (const std::exception& e) {
            ROS_ERROR("Error processing combined point clouds: %s", e.what());
        }
    }
    
    void staticCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(static_cloud_mutex_);
        latest_static_cloud_ = msg;
    }
    
    void dynamicCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(dynamic_cloud_mutex_);
        latest_dynamic_cloud_ = msg;
    }
    
    void lidarOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        latest_odom_ = msg;
    }
    
    void timerCallback(const ros::TimerEvent&) {
        sendCombinedPointClouds();
    }

public:
    PointCloudProcessor() : nh_("~") {
        // Load parameters
        nh_.param<std::string>("server_address", server_address_, "localhost:50051");
        nh_.param<std::string>("static_cloud_topic", static_cloud_topic_, 
                              "/hash_fusion/static_clouds");
        nh_.param<std::string>("dynamic_cloud_topic", dynamic_cloud_topic_, 
                              "/hash_fusion/dynamic_clouds");
        nh_.param<std::string>("lidar_odom_topic", lidar_odom_topic_, 
                              "/lidar/odom");
        nh_.param<double>("publish_rate", publish_rate_, 10.0);  // 10 Hz default
        nh_.param<double>("height_threshold", height_threshold_, 0.5);  // 0.5m default
        
        // Setup gRPC channel and stub
        auto channel = grpc::CreateChannel(server_address_, 
                                         grpc::InsecureChannelCredentials());
        cloud_stub_ = RatSim::PointCloudService::NewStub(channel);
        
        // Setup subscribers
        static_cloud_sub_ = nh_.subscribe(static_cloud_topic_, 5, 
                                        &PointCloudProcessor::staticCloudCallback, 
                                        this);
        dynamic_cloud_sub_ = nh_.subscribe(dynamic_cloud_topic_, 5, 
                                         &PointCloudProcessor::dynamicCloudCallback, 
                                         this);
        lidar_odom_sub_ = nh_.subscribe(lidar_odom_topic_, 5,
                                       &PointCloudProcessor::lidarOdomCallback,
                                       this);
        
        // Setup timer for fixed rate processing
        timer_ = nh_.createTimer(ros::Duration(1.0/publish_rate_), 
                               &PointCloudProcessor::timerCallback, 
                               this);
                                
        ROS_INFO("Point cloud processor initialized:");
        ROS_INFO(" - Server address: %s", server_address_.c_str());
        ROS_INFO(" - Static cloud topic: %s", static_cloud_topic_.c_str());
        ROS_INFO(" - Dynamic cloud topic: %s", dynamic_cloud_topic_.c_str());
        ROS_INFO(" - LIDAR odometry topic: %s", lidar_odom_topic_.c_str());
        ROS_INFO(" - Height threshold: %.2f m", height_threshold_);
        ROS_INFO(" - Publish rate: %.1f Hz", publish_rate_);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_processor");
    PointCloudProcessor processor;
    ros::spin();
    return 0;
}