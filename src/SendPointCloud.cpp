#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
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
    ros::Timer timer_;
    std::unique_ptr<RatSim::PointCloudService::Stub> cloud_stub_;
    
    // Mutex for thread safety
    std::mutex static_cloud_mutex_;
    std::mutex dynamic_cloud_mutex_;
    
    // Store latest point clouds
    sensor_msgs::PointCloud2::ConstPtr latest_static_cloud_;
    sensor_msgs::PointCloud2::ConstPtr latest_dynamic_cloud_;
    
    // ROS Parameters
    std::string server_address_;
    std::string static_cloud_topic_;
    std::string dynamic_cloud_topic_;
    double publish_rate_;
    
    // Colors (RGBA packed into uint32)
    static constexpr uint32_t STATIC_COLOR = 0xFF00FF00;  // Green
    static constexpr uint32_t DYNAMIC_COLOR = 0xFFFF0000;  // Red
    
    void sendCombinedPointClouds() {
        try {
            RatSim::PointCloudWithColor cloud_msg;
            
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
                        point_with_color->set_color(STATIC_COLOR);
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
                        point_with_color->set_color(DYNAMIC_COLOR);
                    }
                }
            }
            
            // Only send if we have points
            if (cloud_msg.data_size() > 0) {
                // Send to server
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
        nh_.param<double>("publish_rate", publish_rate_, 10.0);  // 10 Hz default
        
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
        
        // Setup timer for fixed rate processing
        timer_ = nh_.createTimer(ros::Duration(1.0/publish_rate_), 
                               &PointCloudProcessor::timerCallback, 
                               this);
                                
        ROS_INFO("Point cloud processor initialized:");
        ROS_INFO(" - Server address: %s", server_address_.c_str());
        ROS_INFO(" - Static cloud topic: %s", static_cloud_topic_.c_str());
        ROS_INFO(" - Dynamic cloud topic: %s", dynamic_cloud_topic_.c_str());
        ROS_INFO(" - Publish rate: %.1f Hz", publish_rate_);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_processor");
    PointCloudProcessor processor;
    ros::spin();
    return 0;
}