import grpc
import RatSim_pb2
import RatSim_pb2_grpc

def run():
    target = '172.31.178.18:50996'  # Correct gRPC server address and port
    try:
        with grpc.insecure_channel(target) as channel: # No proxy configuration here
            stub = RatSim_pb2_grpc.LidarServiceStub(channel)
            response = stub.GetLiDARData(RatSim_pb2.LidarRequest())
            for data in response.data:
                print(f"x: {data.x}, y: {data.y}, z: {data.z}, hit: {data.hit}")
            lidar_odom = stub.GetLiDAROdom(RatSim_pb2.LidarRequest())
            pose = lidar_odom.pose
            print(f"Pose: x: {pose.x}, y: {pose.y}, z: {pose.z}")
            
    except grpc.RpcError as e:
        print(f"gRPC Error: {e.code()}: {e.details()}")
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == '__main__':
    run()