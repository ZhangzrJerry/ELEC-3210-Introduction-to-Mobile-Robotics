#!/bin/bash
SCRIPT_DIR=$(cd $(dirname $0); pwd)

export APP_PATH=$SCRIPT_DIR/project3_a_star
export DATA_PATH=$SCRIPT_DIR/dataset

docker compose up -d