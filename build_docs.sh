#!/bin/sh
# NOTE: ensure you've built the docker image first by running
#   `docker build -t esp-docs doc`
docker run --rm -v $PWD:/project -w /project -u $UID -e HOME=/tmp esp-docs ./docker_build_docs.sh
