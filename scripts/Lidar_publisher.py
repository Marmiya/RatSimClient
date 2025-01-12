#!/usr/bin/env python

import grpc
import RatSim_pb2
import RatSim_pb2_grpc
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
import sensor_msgs.point_cloud2 as pc2
import tf.transformations as tf
from math import pi, sqrt

CHANNEL = None
STUB = None
FRAME_ID = 'map'
MIN_DISTANCE = 2.0

def init_stub(server_address='localhost:50051'):
    global CHANNEL, STUB
    CHANNEL = grpc.insecure_channel(server_address)
    STUB = RatSim_pb2_grpc.LidarServiceStub(CHANNEL)
    print(f"[init_stub] Created global channel and stub to {server_address}")

def fetch_synchronized_data():
    if STUB is None:
        print("[fetch_synchronized_data] Error: STUB is None, call init_stub first!")
        return None
    
    try:
        response = STUB.GetLiDARDataAndOdom(RatSim_pb2.EmptyRequest(), timeout=5.0)
        return response
    except grpc.RpcError as e:
        if e.code() == grpc.StatusCode.DEADLINE_EXCEEDED:
            rospy.logwarn("Request timeout - the server might be busy. Retrying...")
        elif e.code() == grpc.StatusCode.UNAVAILABLE:
            rospy.logerr("Server unavailable. Please check if the server is running.")
        else:
            rospy.logerr(f"gRPC Error: {e.code()}: {e.details()}")
        return None
    except Exception as e:
        rospy.logerr(f"An error occurred: {str(e)}")
        return None


def calculate_distance_to_lidar(point_x, point_y, point_z, lidar_x, lidar_y, lidar_z):
    dx = point_x - lidar_x
    dy = point_y - lidar_y
    dz = point_z - lidar_z
    return sqrt(dx*dx + dy*dy + dz*dz)


def create_point_cloud2_only_hit(lidar_data, lidar_pose):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = FRAME_ID

    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)
    ]

    lidar_x = lidar_pose.x / 100.0
    lidar_y = lidar_pose.y / 100.0
    lidar_z = lidar_pose.z / 100.0

    points = []
    for point in lidar_data:

        if point.z > -0.05:

            x = point.x / 100.0
            y = -point.y / 100.0
            z = point.z / 100.0
            
            distance = calculate_distance_to_lidar(x, y, z, lidar_x, -lidar_y, lidar_z)
            if distance >= MIN_DISTANCE:
                points.append([x, y, z])

    point_cloud = pc2.create_cloud(header, fields, points)
    return point_cloud


def create_odom_msg(odom_data):
    odom_msg = Odometry()
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = FRAME_ID
    
    pose = odom_data.pose
    odom_msg.pose.pose.position.x = pose.x / 100.0
    odom_msg.pose.pose.position.y = -pose.y / 100.0
    odom_msg.pose.pose.position.z = pose.z / 100.0
    
    quat = tf.quaternion_from_euler(
        pose.roll * pi / 180, 
        -pose.pitch * pi / 180, 
        -pose.yaw * pi / 180)
    odom_msg.pose.pose.orientation.x = quat[0]
    odom_msg.pose.pose.orientation.y = quat[1]
    odom_msg.pose.pose.orientation.z = quat[2]
    odom_msg.pose.pose.orientation.w = quat[3]
    
    twist = odom_data.twist
    odom_msg.twist.twist.linear.x = twist.linear_x
    odom_msg.twist.twist.linear.y = -twist.linear_y
    odom_msg.twist.twist.linear.z = twist.linear_z
    odom_msg.twist.twist.angular.x = twist.angular_x
    odom_msg.twist.twist.angular.y = -twist.angular_y
    odom_msg.twist.twist.angular.z = twist.angular_z
    
    return odom_msg

def publish_synchronized_data():
    pointcloud_pub = rospy.Publisher('/lidar/point_cloud', PointCloud2, queue_size=10)
    odom_pub = rospy.Publisher('/lidar/odom', Odometry, queue_size=10)
    rate = rospy.Rate(100)  # 10 Hz
    consecutive_failures = 0
    max_consecutive_failures = 3 
    while not rospy.is_shutdown():
        
        current_time = rospy.Time.now()
       
        response = fetch_synchronized_data()

        if response:
            try:
                point_cloud_msg = create_point_cloud2_only_hit(response.data.data, response.odom.pose)
                odom_msg = create_odom_msg(response.odom)
                
                point_cloud_msg.header.stamp = current_time
                odom_msg.header.stamp = current_time
                
                pointcloud_pub.publish(point_cloud_msg)
                odom_pub.publish(odom_msg)
                
                consecutive_failures = 0
            except Exception as e:
                rospy.logerr(f"Error processing data: {str(e)}")
                consecutive_failures += 1
        else:
            consecutive_failures += 1
            
        if consecutive_failures >= max_consecutive_failures:
            rospy.logwarn(f"Multiple consecutive failures ({consecutive_failures}). Waiting for 2 seconds...")
            rospy.sleep(2.0)
            consecutive_failures = 0
            
        rate.sleep()

def shutdown_hook():
    rospy.loginfo("Shutting down node.")
    if CHANNEL:
        CHANNEL.close()

if __name__ == '__main__':
    init_stub('172.31.178.18:50996')
    FRAME_ID = 'world'

    try:
        rospy.init_node('lidar_data_publisher', anonymous=True)
        rospy.on_shutdown(shutdown_hook)
        
        publish_synchronized_data()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass