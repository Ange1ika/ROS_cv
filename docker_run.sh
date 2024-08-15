xhost +;
DOCKER_BUILDKIT=0 docker build -t noetic_full . ; \
docker build -t noetic_full . ; \

docker run -it --name ros --rm -e DISPLAY=:1 \
   --network host --ipc="host"\
   -v /tmp/.X11-unix:/tmp/.X11-unix --gpus all \
   --mount type=bind,source=/home/cogmodel/ROS/docker_data/,target=/docker_data \
   --mount type=bind,source=/home/cogmodel/ROS/resources/data,target=/resources/data \
   --mount type=bind,source=/home/cogmodel/ROS/resources/models,target=/resources/models \
   --mount type=bind,source=/home/cogmodel/ROS/resources/cache_clip,target=/root/.cache/clip \
   --mount type=bind,source=/home/cogmodel/ROS/resources/cache_torch,target=/root/.torch/iopath_cache/ \
   noetic_full
   
   
   
