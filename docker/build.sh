#!/bin/bash
docker build -t fourier-grx-client:latest . -f ./docker/build.Dockerfile

id=$(docker create fourier-grx-client:latest)
docker cp $id:/fourier_grx_client/dist/. ./dist/
docker rm -v $id
