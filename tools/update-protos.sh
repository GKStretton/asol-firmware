#!/bin/bash
# Assumes proto repo is at ../asol-protos/

set -e

cwd=$(pwd)
PROTOS_DIR=../asol-protos

cd $PROTOS_DIR
# build
docker-compose up

cd $cwd
mkdir -p ./src/protos
cp -r $PROTOS_DIR/c/* ./src/protos

# Work-around because `--library ./src/protos/nanopb` in the build script
# was causing an error.
sed -i 's/<pb.h>/"..\/nanopb\/pb.h"/' ./src/protos/machinepb/machine.pb.h