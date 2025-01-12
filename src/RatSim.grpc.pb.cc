// Generated by the gRPC C++ plugin.
// If you make any local change, they will be lost.
// source: RatSim.proto

#include "RatSim.pb.h"
#include "RatSim.grpc.pb.h"

#include <functional>
#include <grpcpp/support/async_stream.h>
#include <grpcpp/support/async_unary_call.h>
#include <grpcpp/impl/channel_interface.h>
#include <grpcpp/impl/client_unary_call.h>
#include <grpcpp/support/client_callback.h>
#include <grpcpp/support/message_allocator.h>
#include <grpcpp/support/method_handler.h>
#include <grpcpp/impl/rpc_service_method.h>
#include <grpcpp/support/server_callback.h>
#include <grpcpp/impl/server_callback_handlers.h>
#include <grpcpp/server_context.h>
#include <grpcpp/impl/service_type.h>
#include <grpcpp/support/sync_stream.h>
namespace RatSim {

static const char* LidarService_method_names[] = {
  "/RatSim.LidarService/GetLiDARData",
  "/RatSim.LidarService/GetLiDAROdom",
  "/RatSim.LidarService/GetLiDARDataAndOdom",
  "/RatSim.LidarService/SendPoints",
};

std::unique_ptr< LidarService::Stub> LidarService::NewStub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options) {
  (void)options;
  std::unique_ptr< LidarService::Stub> stub(new LidarService::Stub(channel, options));
  return stub;
}

LidarService::Stub::Stub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options)
  : channel_(channel), rpcmethod_GetLiDARData_(LidarService_method_names[0], options.suffix_for_stats(),::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  , rpcmethod_GetLiDAROdom_(LidarService_method_names[1], options.suffix_for_stats(),::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  , rpcmethod_GetLiDARDataAndOdom_(LidarService_method_names[2], options.suffix_for_stats(),::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  , rpcmethod_SendPoints_(LidarService_method_names[3], options.suffix_for_stats(),::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  {}

::grpc::Status LidarService::Stub::GetLiDARData(::grpc::ClientContext* context, const ::RatSim::EmptyRequest& request, ::RatSim::LidarData* response) {
  return ::grpc::internal::BlockingUnaryCall< ::RatSim::EmptyRequest, ::RatSim::LidarData, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), rpcmethod_GetLiDARData_, context, request, response);
}

void LidarService::Stub::async::GetLiDARData(::grpc::ClientContext* context, const ::RatSim::EmptyRequest* request, ::RatSim::LidarData* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall< ::RatSim::EmptyRequest, ::RatSim::LidarData, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_GetLiDARData_, context, request, response, std::move(f));
}

void LidarService::Stub::async::GetLiDARData(::grpc::ClientContext* context, const ::RatSim::EmptyRequest* request, ::RatSim::LidarData* response, ::grpc::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create< ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_GetLiDARData_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::RatSim::LidarData>* LidarService::Stub::PrepareAsyncGetLiDARDataRaw(::grpc::ClientContext* context, const ::RatSim::EmptyRequest& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderHelper::Create< ::RatSim::LidarData, ::RatSim::EmptyRequest, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), cq, rpcmethod_GetLiDARData_, context, request);
}

::grpc::ClientAsyncResponseReader< ::RatSim::LidarData>* LidarService::Stub::AsyncGetLiDARDataRaw(::grpc::ClientContext* context, const ::RatSim::EmptyRequest& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncGetLiDARDataRaw(context, request, cq);
  result->StartCall();
  return result;
}

::grpc::Status LidarService::Stub::GetLiDAROdom(::grpc::ClientContext* context, const ::RatSim::EmptyRequest& request, ::RatSim::Odometry* response) {
  return ::grpc::internal::BlockingUnaryCall< ::RatSim::EmptyRequest, ::RatSim::Odometry, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), rpcmethod_GetLiDAROdom_, context, request, response);
}

void LidarService::Stub::async::GetLiDAROdom(::grpc::ClientContext* context, const ::RatSim::EmptyRequest* request, ::RatSim::Odometry* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall< ::RatSim::EmptyRequest, ::RatSim::Odometry, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_GetLiDAROdom_, context, request, response, std::move(f));
}

void LidarService::Stub::async::GetLiDAROdom(::grpc::ClientContext* context, const ::RatSim::EmptyRequest* request, ::RatSim::Odometry* response, ::grpc::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create< ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_GetLiDAROdom_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::RatSim::Odometry>* LidarService::Stub::PrepareAsyncGetLiDAROdomRaw(::grpc::ClientContext* context, const ::RatSim::EmptyRequest& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderHelper::Create< ::RatSim::Odometry, ::RatSim::EmptyRequest, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), cq, rpcmethod_GetLiDAROdom_, context, request);
}

::grpc::ClientAsyncResponseReader< ::RatSim::Odometry>* LidarService::Stub::AsyncGetLiDAROdomRaw(::grpc::ClientContext* context, const ::RatSim::EmptyRequest& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncGetLiDAROdomRaw(context, request, cq);
  result->StartCall();
  return result;
}

::grpc::Status LidarService::Stub::GetLiDARDataAndOdom(::grpc::ClientContext* context, const ::RatSim::EmptyRequest& request, ::RatSim::LidarDataAndOdom* response) {
  return ::grpc::internal::BlockingUnaryCall< ::RatSim::EmptyRequest, ::RatSim::LidarDataAndOdom, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), rpcmethod_GetLiDARDataAndOdom_, context, request, response);
}

void LidarService::Stub::async::GetLiDARDataAndOdom(::grpc::ClientContext* context, const ::RatSim::EmptyRequest* request, ::RatSim::LidarDataAndOdom* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall< ::RatSim::EmptyRequest, ::RatSim::LidarDataAndOdom, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_GetLiDARDataAndOdom_, context, request, response, std::move(f));
}

void LidarService::Stub::async::GetLiDARDataAndOdom(::grpc::ClientContext* context, const ::RatSim::EmptyRequest* request, ::RatSim::LidarDataAndOdom* response, ::grpc::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create< ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_GetLiDARDataAndOdom_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::RatSim::LidarDataAndOdom>* LidarService::Stub::PrepareAsyncGetLiDARDataAndOdomRaw(::grpc::ClientContext* context, const ::RatSim::EmptyRequest& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderHelper::Create< ::RatSim::LidarDataAndOdom, ::RatSim::EmptyRequest, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), cq, rpcmethod_GetLiDARDataAndOdom_, context, request);
}

::grpc::ClientAsyncResponseReader< ::RatSim::LidarDataAndOdom>* LidarService::Stub::AsyncGetLiDARDataAndOdomRaw(::grpc::ClientContext* context, const ::RatSim::EmptyRequest& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncGetLiDARDataAndOdomRaw(context, request, cq);
  result->StartCall();
  return result;
}

::grpc::Status LidarService::Stub::SendPoints(::grpc::ClientContext* context, const ::RatSim::LidarData& request, ::RatSim::Status* response) {
  return ::grpc::internal::BlockingUnaryCall< ::RatSim::LidarData, ::RatSim::Status, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), rpcmethod_SendPoints_, context, request, response);
}

void LidarService::Stub::async::SendPoints(::grpc::ClientContext* context, const ::RatSim::LidarData* request, ::RatSim::Status* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall< ::RatSim::LidarData, ::RatSim::Status, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_SendPoints_, context, request, response, std::move(f));
}

void LidarService::Stub::async::SendPoints(::grpc::ClientContext* context, const ::RatSim::LidarData* request, ::RatSim::Status* response, ::grpc::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create< ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_SendPoints_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::RatSim::Status>* LidarService::Stub::PrepareAsyncSendPointsRaw(::grpc::ClientContext* context, const ::RatSim::LidarData& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderHelper::Create< ::RatSim::Status, ::RatSim::LidarData, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), cq, rpcmethod_SendPoints_, context, request);
}

::grpc::ClientAsyncResponseReader< ::RatSim::Status>* LidarService::Stub::AsyncSendPointsRaw(::grpc::ClientContext* context, const ::RatSim::LidarData& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncSendPointsRaw(context, request, cq);
  result->StartCall();
  return result;
}

LidarService::Service::Service() {
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      LidarService_method_names[0],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< LidarService::Service, ::RatSim::EmptyRequest, ::RatSim::LidarData, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(
          [](LidarService::Service* service,
             ::grpc::ServerContext* ctx,
             const ::RatSim::EmptyRequest* req,
             ::RatSim::LidarData* resp) {
               return service->GetLiDARData(ctx, req, resp);
             }, this)));
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      LidarService_method_names[1],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< LidarService::Service, ::RatSim::EmptyRequest, ::RatSim::Odometry, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(
          [](LidarService::Service* service,
             ::grpc::ServerContext* ctx,
             const ::RatSim::EmptyRequest* req,
             ::RatSim::Odometry* resp) {
               return service->GetLiDAROdom(ctx, req, resp);
             }, this)));
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      LidarService_method_names[2],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< LidarService::Service, ::RatSim::EmptyRequest, ::RatSim::LidarDataAndOdom, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(
          [](LidarService::Service* service,
             ::grpc::ServerContext* ctx,
             const ::RatSim::EmptyRequest* req,
             ::RatSim::LidarDataAndOdom* resp) {
               return service->GetLiDARDataAndOdom(ctx, req, resp);
             }, this)));
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      LidarService_method_names[3],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< LidarService::Service, ::RatSim::LidarData, ::RatSim::Status, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(
          [](LidarService::Service* service,
             ::grpc::ServerContext* ctx,
             const ::RatSim::LidarData* req,
             ::RatSim::Status* resp) {
               return service->SendPoints(ctx, req, resp);
             }, this)));
}

LidarService::Service::~Service() {
}

::grpc::Status LidarService::Service::GetLiDARData(::grpc::ServerContext* context, const ::RatSim::EmptyRequest* request, ::RatSim::LidarData* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}

::grpc::Status LidarService::Service::GetLiDAROdom(::grpc::ServerContext* context, const ::RatSim::EmptyRequest* request, ::RatSim::Odometry* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}

::grpc::Status LidarService::Service::GetLiDARDataAndOdom(::grpc::ServerContext* context, const ::RatSim::EmptyRequest* request, ::RatSim::LidarDataAndOdom* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}

::grpc::Status LidarService::Service::SendPoints(::grpc::ServerContext* context, const ::RatSim::LidarData* request, ::RatSim::Status* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}


static const char* MeshService_method_names[] = {
  "/RatSim.MeshService/SendMesh",
};

std::unique_ptr< MeshService::Stub> MeshService::NewStub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options) {
  (void)options;
  std::unique_ptr< MeshService::Stub> stub(new MeshService::Stub(channel, options));
  return stub;
}

MeshService::Stub::Stub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options)
  : channel_(channel), rpcmethod_SendMesh_(MeshService_method_names[0], options.suffix_for_stats(),::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  {}

::grpc::Status MeshService::Stub::SendMesh(::grpc::ClientContext* context, const ::RatSim::MeshData& request, ::RatSim::Status* response) {
  return ::grpc::internal::BlockingUnaryCall< ::RatSim::MeshData, ::RatSim::Status, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), rpcmethod_SendMesh_, context, request, response);
}

void MeshService::Stub::async::SendMesh(::grpc::ClientContext* context, const ::RatSim::MeshData* request, ::RatSim::Status* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall< ::RatSim::MeshData, ::RatSim::Status, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_SendMesh_, context, request, response, std::move(f));
}

void MeshService::Stub::async::SendMesh(::grpc::ClientContext* context, const ::RatSim::MeshData* request, ::RatSim::Status* response, ::grpc::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create< ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_SendMesh_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::RatSim::Status>* MeshService::Stub::PrepareAsyncSendMeshRaw(::grpc::ClientContext* context, const ::RatSim::MeshData& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderHelper::Create< ::RatSim::Status, ::RatSim::MeshData, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), cq, rpcmethod_SendMesh_, context, request);
}

::grpc::ClientAsyncResponseReader< ::RatSim::Status>* MeshService::Stub::AsyncSendMeshRaw(::grpc::ClientContext* context, const ::RatSim::MeshData& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncSendMeshRaw(context, request, cq);
  result->StartCall();
  return result;
}

MeshService::Service::Service() {
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      MeshService_method_names[0],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< MeshService::Service, ::RatSim::MeshData, ::RatSim::Status, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(
          [](MeshService::Service* service,
             ::grpc::ServerContext* ctx,
             const ::RatSim::MeshData* req,
             ::RatSim::Status* resp) {
               return service->SendMesh(ctx, req, resp);
             }, this)));
}

MeshService::Service::~Service() {
}

::grpc::Status MeshService::Service::SendMesh(::grpc::ServerContext* context, const ::RatSim::MeshData* request, ::RatSim::Status* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}


static const char* DepthCameraService_method_names[] = {
  "/RatSim.DepthCameraService/GetDepthCameraPointData",
  "/RatSim.DepthCameraService/GetDepthCameraImageData",
  "/RatSim.DepthCameraService/GetDepthCameraOdom",
};

std::unique_ptr< DepthCameraService::Stub> DepthCameraService::NewStub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options) {
  (void)options;
  std::unique_ptr< DepthCameraService::Stub> stub(new DepthCameraService::Stub(channel, options));
  return stub;
}

DepthCameraService::Stub::Stub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options)
  : channel_(channel), rpcmethod_GetDepthCameraPointData_(DepthCameraService_method_names[0], options.suffix_for_stats(),::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  , rpcmethod_GetDepthCameraImageData_(DepthCameraService_method_names[1], options.suffix_for_stats(),::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  , rpcmethod_GetDepthCameraOdom_(DepthCameraService_method_names[2], options.suffix_for_stats(),::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  {}

::grpc::Status DepthCameraService::Stub::GetDepthCameraPointData(::grpc::ClientContext* context, const ::RatSim::EmptyRequest& request, ::RatSim::DepthCameraPointData* response) {
  return ::grpc::internal::BlockingUnaryCall< ::RatSim::EmptyRequest, ::RatSim::DepthCameraPointData, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), rpcmethod_GetDepthCameraPointData_, context, request, response);
}

void DepthCameraService::Stub::async::GetDepthCameraPointData(::grpc::ClientContext* context, const ::RatSim::EmptyRequest* request, ::RatSim::DepthCameraPointData* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall< ::RatSim::EmptyRequest, ::RatSim::DepthCameraPointData, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_GetDepthCameraPointData_, context, request, response, std::move(f));
}

void DepthCameraService::Stub::async::GetDepthCameraPointData(::grpc::ClientContext* context, const ::RatSim::EmptyRequest* request, ::RatSim::DepthCameraPointData* response, ::grpc::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create< ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_GetDepthCameraPointData_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::RatSim::DepthCameraPointData>* DepthCameraService::Stub::PrepareAsyncGetDepthCameraPointDataRaw(::grpc::ClientContext* context, const ::RatSim::EmptyRequest& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderHelper::Create< ::RatSim::DepthCameraPointData, ::RatSim::EmptyRequest, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), cq, rpcmethod_GetDepthCameraPointData_, context, request);
}

::grpc::ClientAsyncResponseReader< ::RatSim::DepthCameraPointData>* DepthCameraService::Stub::AsyncGetDepthCameraPointDataRaw(::grpc::ClientContext* context, const ::RatSim::EmptyRequest& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncGetDepthCameraPointDataRaw(context, request, cq);
  result->StartCall();
  return result;
}

::grpc::Status DepthCameraService::Stub::GetDepthCameraImageData(::grpc::ClientContext* context, const ::RatSim::EmptyRequest& request, ::RatSim::DepthCameraImageData* response) {
  return ::grpc::internal::BlockingUnaryCall< ::RatSim::EmptyRequest, ::RatSim::DepthCameraImageData, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), rpcmethod_GetDepthCameraImageData_, context, request, response);
}

void DepthCameraService::Stub::async::GetDepthCameraImageData(::grpc::ClientContext* context, const ::RatSim::EmptyRequest* request, ::RatSim::DepthCameraImageData* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall< ::RatSim::EmptyRequest, ::RatSim::DepthCameraImageData, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_GetDepthCameraImageData_, context, request, response, std::move(f));
}

void DepthCameraService::Stub::async::GetDepthCameraImageData(::grpc::ClientContext* context, const ::RatSim::EmptyRequest* request, ::RatSim::DepthCameraImageData* response, ::grpc::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create< ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_GetDepthCameraImageData_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::RatSim::DepthCameraImageData>* DepthCameraService::Stub::PrepareAsyncGetDepthCameraImageDataRaw(::grpc::ClientContext* context, const ::RatSim::EmptyRequest& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderHelper::Create< ::RatSim::DepthCameraImageData, ::RatSim::EmptyRequest, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), cq, rpcmethod_GetDepthCameraImageData_, context, request);
}

::grpc::ClientAsyncResponseReader< ::RatSim::DepthCameraImageData>* DepthCameraService::Stub::AsyncGetDepthCameraImageDataRaw(::grpc::ClientContext* context, const ::RatSim::EmptyRequest& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncGetDepthCameraImageDataRaw(context, request, cq);
  result->StartCall();
  return result;
}

::grpc::Status DepthCameraService::Stub::GetDepthCameraOdom(::grpc::ClientContext* context, const ::RatSim::EmptyRequest& request, ::RatSim::Odometry* response) {
  return ::grpc::internal::BlockingUnaryCall< ::RatSim::EmptyRequest, ::RatSim::Odometry, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), rpcmethod_GetDepthCameraOdom_, context, request, response);
}

void DepthCameraService::Stub::async::GetDepthCameraOdom(::grpc::ClientContext* context, const ::RatSim::EmptyRequest* request, ::RatSim::Odometry* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall< ::RatSim::EmptyRequest, ::RatSim::Odometry, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_GetDepthCameraOdom_, context, request, response, std::move(f));
}

void DepthCameraService::Stub::async::GetDepthCameraOdom(::grpc::ClientContext* context, const ::RatSim::EmptyRequest* request, ::RatSim::Odometry* response, ::grpc::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create< ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_GetDepthCameraOdom_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::RatSim::Odometry>* DepthCameraService::Stub::PrepareAsyncGetDepthCameraOdomRaw(::grpc::ClientContext* context, const ::RatSim::EmptyRequest& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderHelper::Create< ::RatSim::Odometry, ::RatSim::EmptyRequest, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), cq, rpcmethod_GetDepthCameraOdom_, context, request);
}

::grpc::ClientAsyncResponseReader< ::RatSim::Odometry>* DepthCameraService::Stub::AsyncGetDepthCameraOdomRaw(::grpc::ClientContext* context, const ::RatSim::EmptyRequest& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncGetDepthCameraOdomRaw(context, request, cq);
  result->StartCall();
  return result;
}

DepthCameraService::Service::Service() {
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      DepthCameraService_method_names[0],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< DepthCameraService::Service, ::RatSim::EmptyRequest, ::RatSim::DepthCameraPointData, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(
          [](DepthCameraService::Service* service,
             ::grpc::ServerContext* ctx,
             const ::RatSim::EmptyRequest* req,
             ::RatSim::DepthCameraPointData* resp) {
               return service->GetDepthCameraPointData(ctx, req, resp);
             }, this)));
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      DepthCameraService_method_names[1],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< DepthCameraService::Service, ::RatSim::EmptyRequest, ::RatSim::DepthCameraImageData, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(
          [](DepthCameraService::Service* service,
             ::grpc::ServerContext* ctx,
             const ::RatSim::EmptyRequest* req,
             ::RatSim::DepthCameraImageData* resp) {
               return service->GetDepthCameraImageData(ctx, req, resp);
             }, this)));
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      DepthCameraService_method_names[2],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< DepthCameraService::Service, ::RatSim::EmptyRequest, ::RatSim::Odometry, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(
          [](DepthCameraService::Service* service,
             ::grpc::ServerContext* ctx,
             const ::RatSim::EmptyRequest* req,
             ::RatSim::Odometry* resp) {
               return service->GetDepthCameraOdom(ctx, req, resp);
             }, this)));
}

DepthCameraService::Service::~Service() {
}

::grpc::Status DepthCameraService::Service::GetDepthCameraPointData(::grpc::ServerContext* context, const ::RatSim::EmptyRequest* request, ::RatSim::DepthCameraPointData* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}

::grpc::Status DepthCameraService::Service::GetDepthCameraImageData(::grpc::ServerContext* context, const ::RatSim::EmptyRequest* request, ::RatSim::DepthCameraImageData* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}

::grpc::Status DepthCameraService::Service::GetDepthCameraOdom(::grpc::ServerContext* context, const ::RatSim::EmptyRequest* request, ::RatSim::Odometry* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}


}  // namespace RatSim

