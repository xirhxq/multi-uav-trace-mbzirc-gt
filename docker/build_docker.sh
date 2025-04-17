#!/bin/bash
set -e

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)

IMAGE_NAME="multi-uav-trace-mbzirc-gt"

echo "🚀 Building ${IMAGE_NAME}..."
docker build -t "${IMAGE_NAME}" $SCRIPT_DIR 
echo "✅ Built ${IMAGE_NAME}"
