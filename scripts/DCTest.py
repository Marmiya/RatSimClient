import grpc
import RatSim_pb2
import RatSim_pb2_grpc

def test_depth_camera_service():
    # Create gRPC channel with increased message size
    options = [
        ('grpc.max_receive_message_length', 10 * 1024 * 1024),  # 10MB
        ('grpc.max_send_message_length', 10 * 1024 * 1024)      # 10MB
    ]
    channel = grpc.insecure_channel(
        '172.31.178.18:50996',
        options=options
    )
    
    # Rest of your code remains the same
    stub = RatSim_pb2_grpc.DepthCameraServiceStub(channel)
    
    try:
        request = RatSim_pb2.RobotName(name="Mavic")
        
        print("Testing GetDepthCameraPointData...")
        point_data = stub.GetDepthCameraPointData(request)
        print(f"Received point data with {len(point_data.data)} points")
        if len(point_data.data) > 0:
            print(f"Sample point: x={point_data.data[0].x}, y={point_data.data[0].y}, z={point_data.data[0].z}")
        
        print("\nTesting GetDepthCameraImageData...")
        image_data = stub.GetDepthCameraImageData(request)
        print(f"Received image data with {len(image_data.data)} depth values")
        if len(image_data.data) > 0:
            print(f"Sample depth value: {image_data.data[0]}")
        
        print("\nTesting GetDepthCameraOdom...")
        odom = stub.GetDepthCameraOdom(request)
        print(f"Received odometry:")
        print(f"Position: x={odom.pose.x}, y={odom.pose.y}, z={odom.pose.z}")
        print(f"Rotation: roll={odom.pose.roll}, pitch={odom.pose.pitch}, yaw={odom.pose.yaw}")
        
    except grpc.RpcError as e:
        print(f"RPC error: {e.code()}: {e.details()}")
    finally:
        channel.close()

if __name__ == "__main__":
    test_depth_camera_service()