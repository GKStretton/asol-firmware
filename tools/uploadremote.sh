#!/bin/bash
# upload via mqtt and pygateway

set -e

./build.sh
mosquitto_pub -h DEPTH -t mega/flash -m "$(cat ./build/Light.ino.hex)"

echo "firmware sent to mega/flash"
