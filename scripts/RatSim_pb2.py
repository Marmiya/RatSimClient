# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# NO CHECKED-IN PROTOBUF GENCODE
# source: RatSim.proto
# Protobuf Python Version: 5.29.0
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import runtime_version as _runtime_version
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
_runtime_version.ValidateProtobufRuntimeVersion(
    _runtime_version.Domain.PUBLIC,
    5,
    29,
    0,
    '',
    'RatSim.proto'
)
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x0cRatSim.proto\x12\x06RatSim\"(\n\x05Point\x12\t\n\x01x\x18\x01 \x01(\x02\x12\t\n\x01y\x18\x02 \x01(\x02\x12\t\n\x01z\x18\x03 \x01(\x02\"\x0e\n\x0c\x45mptyRequest\"D\n\x08Odometry\x12\x1a\n\x04pose\x18\x01 \x01(\x0b\x32\x0c.RatSim.Pose\x12\x1c\n\x05twist\x18\x02 \x01(\x0b\x32\r.RatSim.twist\"\x18\n\x06Status\x12\x0e\n\x06status\x18\x01 \x01(\x08\"n\n\x08MeshData\x12\x0c\n\x04\x64\x61ta\x18\x01 \x01(\x0c\x12\x0e\n\x06\x66ormat\x18\x02 \x01(\r\x12\x0f\n\x07version\x18\x03 \x01(\r\x12\x12\n\nsimplified\x18\x04 \x01(\x08\x12\x1f\n\ttransform\x18\x05 \x01(\x0b\x32\x0c.RatSim.Pose\"R\n\x10MeshDataWithOdom\x12\x1e\n\x04mesh\x18\x01 \x01(\x0b\x32\x10.RatSim.MeshData\x12\x1e\n\x04odom\x18\x02 \x01(\x0b\x32\x10.RatSim.Odometry\"Q\n\x04Pose\x12\t\n\x01x\x18\x01 \x01(\x02\x12\t\n\x01y\x18\x02 \x01(\x02\x12\t\n\x01z\x18\x03 \x01(\x02\x12\x0c\n\x04roll\x18\x04 \x01(\x02\x12\r\n\x05pitch\x18\x05 \x01(\x02\x12\x0b\n\x03yaw\x18\x06 \x01(\x02\"v\n\x05twist\x12\x10\n\x08linear_x\x18\x01 \x01(\x02\x12\x10\n\x08linear_y\x18\x02 \x01(\x02\x12\x10\n\x08linear_z\x18\x03 \x01(\x02\x12\x11\n\tangular_x\x18\x04 \x01(\x02\x12\x11\n\tangular_y\x18\x05 \x01(\x02\x12\x11\n\tangular_z\x18\x06 \x01(\x02\":\n\nLidarPoint\x12\t\n\x01x\x18\x01 \x01(\x02\x12\t\n\x01y\x18\x02 \x01(\x02\x12\t\n\x01z\x18\x03 \x01(\x02\x12\x0b\n\x03hit\x18\x04 \x01(\x05\"-\n\tLidarData\x12 \n\x04\x64\x61ta\x18\x01 \x03(\x0b\x32\x12.RatSim.LidarPoint\"S\n\x10LidarDataAndOdom\x12\x1f\n\x04\x64\x61ta\x18\x01 \x01(\x0b\x32\x11.RatSim.LidarData\x12\x1e\n\x04odom\x18\x02 \x01(\x0b\x32\x10.RatSim.Odometry\"3\n\x14\x44\x65pthCameraPointData\x12\x1b\n\x04\x64\x61ta\x18\x01 \x03(\x0b\x32\r.RatSim.Point\"$\n\x14\x44\x65pthCameraImageData\x12\x0c\n\x04\x64\x61ta\x18\x01 \x03(\x02\x32\xf7\x01\n\x0cLidarService\x12\x37\n\x0cGetLiDARData\x12\x14.RatSim.EmptyRequest\x1a\x11.RatSim.LidarData\x12\x36\n\x0cGetLiDAROdom\x12\x14.RatSim.EmptyRequest\x1a\x10.RatSim.Odometry\x12\x45\n\x13GetLiDARDataAndOdom\x12\x14.RatSim.EmptyRequest\x1a\x18.RatSim.LidarDataAndOdom\x12/\n\nSendPoints\x12\x11.RatSim.LidarData\x1a\x0e.RatSim.Status2;\n\x0bMeshService\x12,\n\x08SendMesh\x12\x10.RatSim.MeshData\x1a\x0e.RatSim.Status2\xf0\x01\n\x12\x44\x65pthCameraService\x12M\n\x17GetDepthCameraPointData\x12\x14.RatSim.EmptyRequest\x1a\x1c.RatSim.DepthCameraPointData\x12M\n\x17GetDepthCameraImageData\x12\x14.RatSim.EmptyRequest\x1a\x1c.RatSim.DepthCameraImageData\x12<\n\x12GetDepthCameraOdom\x12\x14.RatSim.EmptyRequest\x1a\x10.RatSim.Odometryb\x06proto3')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'RatSim_pb2', _globals)
if not _descriptor._USE_C_DESCRIPTORS:
  DESCRIPTOR._loaded_options = None
  _globals['_POINT']._serialized_start=24
  _globals['_POINT']._serialized_end=64
  _globals['_EMPTYREQUEST']._serialized_start=66
  _globals['_EMPTYREQUEST']._serialized_end=80
  _globals['_ODOMETRY']._serialized_start=82
  _globals['_ODOMETRY']._serialized_end=150
  _globals['_STATUS']._serialized_start=152
  _globals['_STATUS']._serialized_end=176
  _globals['_MESHDATA']._serialized_start=178
  _globals['_MESHDATA']._serialized_end=288
  _globals['_MESHDATAWITHODOM']._serialized_start=290
  _globals['_MESHDATAWITHODOM']._serialized_end=372
  _globals['_POSE']._serialized_start=374
  _globals['_POSE']._serialized_end=455
  _globals['_TWIST']._serialized_start=457
  _globals['_TWIST']._serialized_end=575
  _globals['_LIDARPOINT']._serialized_start=577
  _globals['_LIDARPOINT']._serialized_end=635
  _globals['_LIDARDATA']._serialized_start=637
  _globals['_LIDARDATA']._serialized_end=682
  _globals['_LIDARDATAANDODOM']._serialized_start=684
  _globals['_LIDARDATAANDODOM']._serialized_end=767
  _globals['_DEPTHCAMERAPOINTDATA']._serialized_start=769
  _globals['_DEPTHCAMERAPOINTDATA']._serialized_end=820
  _globals['_DEPTHCAMERAIMAGEDATA']._serialized_start=822
  _globals['_DEPTHCAMERAIMAGEDATA']._serialized_end=858
  _globals['_LIDARSERVICE']._serialized_start=861
  _globals['_LIDARSERVICE']._serialized_end=1108
  _globals['_MESHSERVICE']._serialized_start=1110
  _globals['_MESHSERVICE']._serialized_end=1169
  _globals['_DEPTHCAMERASERVICE']._serialized_start=1172
  _globals['_DEPTHCAMERASERVICE']._serialized_end=1412
# @@protoc_insertion_point(module_scope)
