#!/bin/bash
# Assumes proto repo is at ../asol-protos/

set -e

cwd=$(pwd)
EXTRAS_DIR=../asol-protos

cd $EXTRAS_DIR
# build
docker-compose up

cd $cwd
mkdir -p ./src/extras/
cp -r $EXTRAS_DIR/c/* ./src/extras/

# Work-around because `--library ./src/extras/nanopb` in the build script
# was causing an error.
sed -i 's/<pb.h>/"..\/nanopb\/pb.h"/' ./src/extras/machinepb/machine.pb.h