#!/usr/bin/env python

import grpc
import RatSim_pb2
import RatSim_pb2_grpc
import rospy

CHANNEL = None
STUB = None
DRONE_NAME = 'test_drone'

def init_stub(server_address='localhost:50051'):
    global CHANNEL, STUB
    CHANNEL = grpc.insecure_channel(server_address)
    STUB = RatSim_pb2_grpc.DroneServiceStub(CHANNEL)
    print(f"[init_stub] Created global channel and stub to {server_address}")

def send_drone_pose(x, y, z, roll, pitch, yaw):
    """Send drone pose to the service"""
    if STUB is None:
        print("[send_drone_pose] Error: STUB is None, call init_stub first!")
        return False
    
    try:
        # Create DronePose message
        drone_pose = RatSim_pb2.DronePose()
        drone_pose.name = DRONE_NAME
        drone_pose.pose.x = x
        drone_pose.pose.y = y
        drone_pose.pose.z = z
        drone_pose.pose.roll = roll
        drone_pose.pose.pitch = pitch
        drone_pose.pose.yaw = yaw
        
        response = STUB.SendDronePose(drone_pose, timeout=5.0)
        return response.status
    except grpc.RpcError as e:
        if e.code() == grpc.StatusCode.DEADLINE_EXCEEDED:
            rospy.logwarn("Request timeout - the server might be busy.")
        elif e.code() == grpc.StatusCode.UNAVAILABLE:
            rospy.logerr("Server unavailable. Please check if the server is running.")
        else:
            rospy.logerr(f"gRPC Error: {e.code()}: {e.details()}")
        return False
    except Exception as e:
        rospy.logerr(f"An error occurred: {str(e)}")
        return False

def shutdown_hook():
    rospy.loginfo("Shutting down node.")
    if CHANNEL:
        CHANNEL.close()

def main():
    global DRONE_NAME
    
    # Initialize ROS node
    rospy.init_node('drone_pose_publisher', anonymous=True)
    
    # Get parameters from ROS parameter server
    server_address = rospy.get_param('~server_address', '172.31.178.18:50996')
    DRONE_NAME = rospy.get_param('~drone_name', 'test_drone')
    rate_hz = rospy.get_param('~rate', 10)  # Publishing rate in Hz
    
    # Get pose parameters
    x = rospy.get_param('~x', 0.0)
    y = rospy.get_param('~y', 0.0)
    z = rospy.get_param('~z', 0.0)
    roll = rospy.get_param('~roll', 0.0)
    pitch = rospy.get_param('~pitch', 0.0)
    yaw = rospy.get_param('~yaw', 0.0)
    
    # Initialize gRPC stub
    init_stub(server_address)
    
    # Register shutdown hook
    rospy.on_shutdown(shutdown_hook)
    
    # Create rate object
    rate = rospy.Rate(rate_hz)
    
    # Main loop
    while not rospy.is_shutdown():
        success = send_drone_pose(x, y, z, roll, pitch, yaw)
        if not success:
            rospy.logwarn("Failed to send drone pose")
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass