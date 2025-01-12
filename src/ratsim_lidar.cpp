#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2/LinearMath/Quaternion.h>
#include <grpc++/grpc++.h>
#include "RatSim.grpc.pb.h"

class LidarDataPublisher {
private:
    ros::NodeHandle nh_;
    ros::Publisher pointcloud_pub_;
    ros::Publisher odom_pub_;
    std::unique_ptr<RatSim::LidarService::Stub> stub_;
    std::shared_ptr<grpc::Channel> channel_;
    std::string frame_id_;
    const double MIN_DISTANCE = 2.0;
    const int MAX_CONSECUTIVE_FAILURES = 3;
    int rate_ = 10;

public:
    LidarDataPublisher() : frame_id_("world") {
        ros::NodeHandle private_nh("~"); // Create private node handle
        pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/lidar/point_cloud", 1);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/lidar/odom", 1);
        private_nh.param("rate", rate_, 10); // Use private node handle to get parameter
        ROS_INFO_STREAM("Rate: " << rate_);
    }

    bool initStub(const std::string& server_address = "localhost:50051") {
        channel_ = grpc::CreateChannel(server_address, grpc::InsecureChannelCredentials());
        stub_ = RatSim::LidarService::NewStub(channel_);
        ROS_INFO_STREAM("Created channel and stub to " << server_address);
        return true;
    }

    bool fetchSynchronizedData(RatSim::LidarDataAndOdom& response) {
        if (!stub_) {
            ROS_ERROR("Error: stub_ is null, call initStub first!");
            return false;
        }

        grpc::ClientContext context;
        RatSim::EmptyRequest request;
        context.set_deadline(std::chrono::system_clock::now() + std::chrono::seconds(5));

        grpc::Status status = stub_->GetLiDARDataAndOdom(&context, request, &response);

        if (!status.ok()) {
            if (status.error_code() == grpc::StatusCode::DEADLINE_EXCEEDED) {
                ROS_WARN("Request timeout - the server might be busy. Retrying...");
            } else if (status.error_code() == grpc::StatusCode::UNAVAILABLE) {
                ROS_ERROR("Server unavailable. Please check if the server is running.");
            } else {
                ROS_ERROR_STREAM("gRPC Error: " << status.error_code() << ": " << status.error_message());
            }
            return false;
        }
        return true;
    }

    double calculateDistanceToLidar(double point_x, double point_y, double point_z,
                                  double lidar_x, double lidar_y, double lidar_z) {
        double dx = point_x - lidar_x;
        double dy = point_y - lidar_y;
        double dz = point_z - lidar_z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    sensor_msgs::PointCloud2 createPointCloud2OnlyHit(
    const google::protobuf::RepeatedPtrField<RatSim::LidarPoint>& lidar_data,
    const RatSim::Pose& lidar_pose) {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    double lidar_x = lidar_pose.x() / 100.0;
    double lidar_y = lidar_pose.y() / 100.0;
    double lidar_z = lidar_pose.z() / 100.0;

    for (const auto& point : lidar_data) {
        // Only process points that have hit something (using the hit field from LidarPoint)
        if (point.hit() && point.z() > -0.05) {
            double x = point.x() / 100.0;
            double y = -point.y() / 100.0;
            double z = point.z() / 100.0;

            double distance = calculateDistanceToLidar(x, y, z, lidar_x, -lidar_y, lidar_z);
            if (distance >= MIN_DISTANCE) {
                pcl::PointXYZ pcl_point;
                pcl_point.x = x;
                pcl_point.y = y;
                pcl_point.z = z;
                cloud->points.push_back(pcl_point);
            }
        }
    }

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.frame_id = frame_id_;
    
    return cloud_msg;
}

    nav_msgs::Odometry createOdomMsg(const RatSim::Odometry& odom_data) {
        nav_msgs::Odometry odom_msg;
        odom_msg.header.frame_id = frame_id_;
        
        const auto& pose = odom_data.pose();
        odom_msg.pose.pose.position.x = pose.x() / 100.0;
        odom_msg.pose.pose.position.y = -pose.y() / 100.0;
        odom_msg.pose.pose.position.z = pose.z() / 100.0;
        
        tf2::Quaternion quat;
        quat.setRPY(
            pose.roll() * M_PI / 180.0,
            -pose.pitch() * M_PI / 180.0,
            -pose.yaw() * M_PI / 180.0
        );
        
        odom_msg.pose.pose.orientation.x = quat.x();
        odom_msg.pose.pose.orientation.y = quat.y();
        odom_msg.pose.pose.orientation.z = quat.z();
        odom_msg.pose.pose.orientation.w = quat.w();
        
        const auto& twist = odom_data.twist();
        odom_msg.twist.twist.linear.x = twist.linear_x();
        odom_msg.twist.twist.linear.y = -twist.linear_y();
        odom_msg.twist.twist.linear.z = twist.linear_z();
        odom_msg.twist.twist.angular.x = twist.angular_x();
        odom_msg.twist.twist.angular.y = -twist.angular_y();
        odom_msg.twist.twist.angular.z = twist.angular_z();
        
        return odom_msg;
    }

    void publishSynchronizedData() {
        ros::Rate rate(rate_);
        int consecutive_failures = 0;
        
        while (ros::ok()) {
            ros::Time current_time = ros::Time::now();
            RatSim::LidarDataAndOdom response;
            
            if (fetchSynchronizedData(response)) {
                try {
                    auto point_cloud_msg = createPointCloud2OnlyHit(
                        response.data().data(), response.odom().pose());
                    auto odom_msg = createOdomMsg(response.odom());
                    
                    point_cloud_msg.header.stamp = current_time;
                    odom_msg.header.stamp = current_time;
                    
                    pointcloud_pub_.publish(point_cloud_msg);
                    odom_pub_.publish(odom_msg);
                    
                    consecutive_failures = 0;
                }
                catch (const std::exception& e) {
                    ROS_ERROR_STREAM("Error processing data: " << e.what());
                    consecutive_failures++;
                }
            }
            else {
                consecutive_failures++;
            }
            
            if (consecutive_failures >= MAX_CONSECUTIVE_FAILURES) {
                ROS_WARN_STREAM("Multiple consecutive failures (" << consecutive_failures 
                               << "). Waiting for 2 seconds...");
                ros::Duration(2.0).sleep();
                consecutive_failures = 0;
            }
            
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_data_publisher");
    
    LidarDataPublisher publisher;
    publisher.initStub("172.31.178.18:50996");
    
    try {
        publisher.publishSynchronizedData();
    }
    catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception in main: " << e.what());
        return 1;
    }
    
    return 0;
}