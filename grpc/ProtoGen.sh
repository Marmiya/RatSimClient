#!/usr/bin/env zsh

# Set paths (adjust if necessary)
PROTOC_PATH="/root/vcpkg/installed/x64-linux/tools/protobuf/protoc"
PLUGIN_PATH="/root/vcpkg/installed/x64-linux/tools/grpc/grpc_cpp_plugin"
PROTO_PATH="." # Current directory
RATSIM_PATH="/root/catkin_ws/src/RatSim" # Path to RatSim package
OUTPUT_HEADER_PATH="$RATSIM_PATH/include/RatSim" # Output directory for headers
OUTPUT_CPP_PATH="$RATSIM_PATH/src" # Output directory for .cc files
OUTPUT_PYTHON_PATH="$RATSIM_PATH/scripts" # Output directory for Python files

# Create output directories if they don't exist
mkdir -p "$OUTPUT_HEADER_PATH"
mkdir -p "$OUTPUT_CPP_PATH"
mkdir -p "$OUTPUT_PYTHON_PATH"

# Create a temporary directory for generation
TMP_DIR=$(mktemp -d)

# Generate all C++ code to temporary directory first
"$PROTOC_PATH" \
    -I="$PROTO_PATH" \
    --grpc_out="$TMP_DIR" \
    --cpp_out="$TMP_DIR" \
    --plugin=protoc-gen-grpc="$PLUGIN_PATH" \
    "$PROTO_PATH/RatSim.proto"

# Move files to their correct locations
mv "$TMP_DIR"/*.h "$OUTPUT_HEADER_PATH"
mv "$TMP_DIR"/*.cc "$OUTPUT_CPP_PATH"

# Clean up temporary directory
rm -rf "$TMP_DIR"

# Generate Python code
python3 -m grpc_tools.protoc \
    -I "$PROTO_PATH" \
    --grpc_python_out="$OUTPUT_PYTHON_PATH" \
    --python_out="$OUTPUT_PYTHON_PATH" \
    "$PROTO_PATH/RatSim.proto"

echo "Proto generation completed."