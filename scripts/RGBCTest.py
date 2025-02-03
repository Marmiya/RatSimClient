import grpc
import RatSim_pb2
import RatSim_pb2_grpc
from PIL import Image
import numpy as np

def test_rgb_camera_service():
    options = [
        ('grpc.max_receive_message_length', 50 * 1024 * 1024),  # 50MB
        ('grpc.max_send_message_length', 50 * 1024 * 1024)      # 50MB
    ]
    channel = grpc.insecure_channel(
        '172.31.178.18:50996',
        options=options
    )
    
    stub = RatSim_pb2_grpc.RGBCameraServiceStub(channel)
    
    try:
        request = RatSim_pb2.RobotName(name="Mavic")
        
        # Test GetRGBImageData
        print("\nTesting GetRGBImageData...")
        image_data = stub.GetRGBCameraImageData(request)
        print(f"Received RGB image data:")
        print(f"Width: {image_data.width}")
        print(f"Height: {image_data.height}")
        print(f"Data size: {len(image_data.data)} bytes")
        
        # Convert and save as PNG
        if image_data.format == RatSim_pb2.RGBCameraImageData.RGB:
            img_array = np.frombuffer(image_data.data, dtype=np.uint8)
            img_array = img_array.reshape((image_data.height, image_data.width, 3))
            img = Image.fromarray(img_array, 'RGB')
            img.save('rgb_camera_image.png')
            print("Saved RGB image as 'rgb_camera_image.png'")
        
        # Test GetRGBCameraOdom
        print("\nTesting GetRGBCameraOdom...")
        odom = stub.GetRGBCameraOdom(request)
        print("Received odometry:")
        print(f"Position: x={odom.pose.x:.3f}, y={odom.pose.y:.3f}, z={odom.pose.z:.3f}")
        print(f"Rotation: roll={odom.pose.roll:.3f}, pitch={odom.pose.pitch:.3f}, yaw={odom.pose.yaw:.3f}")

    except grpc.RpcError as e:
        print(f"RPC error: {e.code()}: {e.details()}")
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        channel.close()

if __name__ == "__main__":
    test_rgb_camera_service()