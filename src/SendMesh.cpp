#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <grpc++/grpc++.h>
#include "RatSim.grpc.pb.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/simplification_remove_unused_vertices.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_quadric_decimation.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>
#include <vector>
#include <array>
#include <tf/tf.h>

// Define custom binary mesh format
struct CompactVertex {
    float x, y, z;  // 12 bytes
};

struct MeshHeader {
    uint32_t magic;         // Magic number to identify format (4 bytes)
    uint32_t version;       // Format version (4 bytes)
    uint32_t vertexCount;   // Number of vertices (4 bytes)
    uint32_t indexCount;    // Number of indices (4 bytes)
    uint32_t flags;         // Bit flags for mesh properties (4 bytes)
};

class MeshProcessor {
private:
    ros::NodeHandle nh_;
    ros::Subscriber mesh_sub_;
    std::unique_ptr<RatSim::MeshService::Stub> mesh_stub_;
    
    // ROS Parameters
    bool enable_simplification_;
    double simplification_factor_;
    std::string server_address_;
    std::string mesh_topic_;
    bool use_quantization_;
    
    static constexpr uint32_t MESH_MAGIC = 0x48534D55;  // "UMSH" in ASCII
    static constexpr uint32_t MESH_VERSION = 1;
    static constexpr uint32_t FLAG_QUANTIZED = 1 << 0;
    
    std::vector<uint8_t> serializeMesh(const std::vector<CompactVertex>& vertices, 
                                     const std::vector<uint32_t>& indices,
                                     bool is_quantized) {
        MeshHeader header{
            MESH_MAGIC,
            MESH_VERSION,
            static_cast<uint32_t>(vertices.size()),
            static_cast<uint32_t>(indices.size()),
            is_quantized ? FLAG_QUANTIZED : 0
        };
        
        size_t total_size = sizeof(MeshHeader) + 
                           vertices.size() * sizeof(CompactVertex) +
                           indices.size() * sizeof(uint32_t);
                           
        std::vector<uint8_t> buffer(total_size);
        size_t offset = 0;
        
        memcpy(buffer.data() + offset, &header, sizeof(MeshHeader));
        offset += sizeof(MeshHeader);
        
        memcpy(buffer.data() + offset, vertices.data(), 
               vertices.size() * sizeof(CompactVertex));
        offset += vertices.size() * sizeof(CompactVertex);
        
        memcpy(buffer.data() + offset, indices.data(), 
               indices.size() * sizeof(uint32_t));
        
        return buffer;
    }
    
    std::pair<std::vector<CompactVertex>, std::vector<uint32_t>>
    convertMarkerToIndexed(const visualization_msgs::Marker& marker_msg) {
        std::vector<CompactVertex> unique_vertices;
        std::vector<uint32_t> indices;
        std::map<std::array<float, 3>, uint32_t> vertex_map;
        
        for (size_t i = 0; i < marker_msg.points.size(); ++i) {
            const auto& p = marker_msg.points[i];
            std::array<float, 3> vertex = { static_cast<float>(p.x), 
                                            static_cast<float>(p.y), 
                                            static_cast<float>(p.z)};
            
            auto it = vertex_map.find(vertex);
            if (it == vertex_map.end()) {
                uint32_t index = static_cast<uint32_t>(unique_vertices.size());
                vertex_map[vertex] = index;
                unique_vertices.push_back({ static_cast<float>(p.x), 
                                            static_cast<float>(p.y), 
                                            static_cast<float>(p.z)});
                indices.push_back(index);
            } else {
                indices.push_back(it->second);
            }
        }
        
        return {unique_vertices, indices};
    }
    
    std::vector<CompactVertex> simplifyVertices(
        const std::vector<CompactVertex>& vertices,
        const std::vector<uint32_t>& indices) {
            
        if (!enable_simplification_) {
            return vertices;
        }
        
        pcl::PolygonMesh pcl_mesh;
        pcl::PointCloud<pcl::PointXYZ> cloud;
        
        for (const auto& v : vertices) {
            pcl::PointXYZ point;
            point.x = v.x;
            point.y = v.y;
            point.z = v.z;
            cloud.points.push_back(point);
        }
        
        pcl::toPCLPointCloud2(cloud, pcl_mesh.cloud);
        
        for (size_t i = 0; i < indices.size(); i += 3) {
            pcl::Vertices polygon;
            polygon.vertices.push_back(indices[i]);
            polygon.vertices.push_back(indices[i + 1]);
            polygon.vertices.push_back(indices[i + 2]);
            pcl_mesh.polygons.push_back(polygon);
        }
        
        pcl::PolygonMesh output_mesh;
        pcl::PolygonMeshPtr input_mesh_ptr(new pcl::PolygonMesh(pcl_mesh));
        pcl::MeshQuadricDecimationVTK decimator;
        decimator.setInputMesh(input_mesh_ptr);
        decimator.setTargetReductionFactor(simplification_factor_);
        decimator.process(output_mesh);
        
        std::vector<CompactVertex> simplified_vertices;
        pcl::PointCloud<pcl::PointXYZ> simplified_cloud;
        pcl::fromPCLPointCloud2(output_mesh.cloud, simplified_cloud);
        
        for (const auto& p : simplified_cloud.points) {
            simplified_vertices.push_back({p.x, p.y, p.z});
        }
        
        return simplified_vertices;
    }

public:
    MeshProcessor() : nh_("~") {
        // Load parameters
        nh_.param<bool>("enable_simplification", enable_simplification_, false);
        nh_.param<double>("simplification_factor", simplification_factor_, 0.5);
        nh_.param<std::string>("server_address", server_address_, "localhost:50051");
        nh_.param<std::string>("mesh_topic", mesh_topic_, "/frame_recon/frame_meshs");
        nh_.param<bool>("use_quantization", use_quantization_, false);
        
        // Setup gRPC channel and stub
        auto channel = grpc::CreateChannel(server_address_, 
                                         grpc::InsecureChannelCredentials());
        mesh_stub_ = RatSim::MeshService::NewStub(channel);
        
        // Setup subscriber
        mesh_sub_ = nh_.subscribe(mesh_topic_, 5, &MeshProcessor::meshCallback, this);
                                
        ROS_INFO("Mesh processor initialized:");
        ROS_INFO(" - Server address: %s", server_address_.c_str());
        ROS_INFO(" - Mesh topic: %s", mesh_topic_.c_str());
        ROS_INFO(" - Mesh simplification: %s", 
                 enable_simplification_ ? "enabled" : "disabled");
        if (enable_simplification_) {
            ROS_INFO(" - Simplification factor: %.2f", simplification_factor_);
        }
        ROS_INFO(" - Vertex quantization: %s", 
                 use_quantization_ ? "enabled" : "disabled");
    }
    
    void meshCallback(const visualization_msgs::Marker::ConstPtr& msg) {
        if (msg->type != visualization_msgs::Marker::TRIANGLE_LIST) {
            ROS_WARN("Received non-triangle list marker. Skipping...");
            return;
        }
        
        try {
            // Convert to indexed format
            auto [vertices, indices] = convertMarkerToIndexed(*msg);
            
            // Simplify if enabled
            auto final_vertices = simplifyVertices(vertices, indices);
            
            // Serialize to binary format
            auto binary_mesh = serializeMesh(final_vertices, indices, use_quantization_);
            
            // Create MeshData message
            RatSim::MeshData mesh_data;
            mesh_data.set_data(binary_mesh.data(), binary_mesh.size());
            mesh_data.set_format(MESH_MAGIC);
            mesh_data.set_version(MESH_VERSION);
            mesh_data.set_simplified(enable_simplification_);
            
            // Set transform from marker pose
            auto* transform = mesh_data.mutable_transform();
            transform->set_x(msg->pose.position.x);
            transform->set_y(msg->pose.position.y);
            transform->set_z(msg->pose.position.z);
            
            // Convert quaternion to euler angles
            double roll, pitch, yaw;
            tf::Quaternion tfQuat;
            tf::quaternionMsgToTF(msg->pose.orientation, tfQuat);
            tf::Matrix3x3(tfQuat).getRPY(roll, pitch, yaw);
            
            transform->set_roll(roll);
            transform->set_pitch(pitch);
            transform->set_yaw(yaw);
            
            // Send to server
            grpc::ClientContext context;
            RatSim::Status response;
            grpc::Status status = mesh_stub_->SendMesh(&context, mesh_data, &response);
            
            if (status.ok() && response.status()) {
                // ROS_INFO("Successfully sent mesh data (%.2f KB)", 
                //         binary_mesh.size() / 1024.0f);
            } else {
                ROS_ERROR("Failed to send mesh data: %s", 
                         status.ok() ? "Server rejected data" : 
                         status.error_message().c_str());
            }
            
        } catch (const std::exception& e) {
            ROS_ERROR("Error processing mesh: %s", e.what());
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mesh_processor");
    MeshProcessor processor;
    ros::spin();
    return 0;
}