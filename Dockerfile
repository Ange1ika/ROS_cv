FROM nvidia/cuda:11.7.1-devel-ubuntu20.04

RUN chmod 1777 /tmp
ENV TZ=Etc/UTC
ARG DEBIAN_FRONTEND=noninteractive 
RUN apt-get update && apt-get install -y lsb-core curl
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Добавление репозитория aruco_ros и установка
RUN apt-get update && apt-get install -y
    
RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full \
    ros-noetic-cv-bridge \
#    ros-noetic-image-transport \
#    ros-noetic-vision-opencv \
    mesa-utils \
    libgl1-mesa-glx \
    python-dev \
    build-essential wget git

ENV CONDA_DIR /opt/conda
ENV PATH=$CONDA_DIR/bin:$PATH
ENV PIP_ROOT_USER_ACTION=ignore
RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh && \
    /bin/bash ~/miniconda.sh -b -p /opt/conda

RUN conda install python=3.8.10

RUN pip install torch==2.0.0+cu117 torchvision==0.15.1+cu117  --index-url https://download.pytorch.org/whl/cu117
ADD requirements.txt ./

RUN pip install Cython
RUN pip install -r requirements.txt
RUN pip install git+https://github.com/openai/CLIP.git
RUN pip install open3d



ADD openseed_src/requirements.txt ./
RUN python -m pip install 'git+https://github.com/MaureenZOU/detectron2-xyz.git'
RUN pip install git+https://github.com/cocodataset/panopticapi.git
RUN python -m pip install -r requirements.txt
RUN pip install albumentations Pillow==9.5.0 wandb open_clip_torch

RUN conda install -c conda-forge rospkg
RUN apt-get -y install software-properties-common 

# ADD init_clip_model.py /sources/init_clip_model.py
# RUN python /sources/init_clip_model.py

# git clone https://github.com/andrey1908/kas_utils.git
ADD kas_utils/ /sources/kas_utils/

# git clone https://github.com/andrey1908/BoT-SORT
ADD BoT-SORT/ /sources/BoT-SORT/ 

# ADD rviz_conf.rviz /sources/rviz_conf.rviz
ADD rviz_conf_full.rviz /sources/rviz_conf.rviz

WORKDIR /sources/kas_utils/python
RUN pip install .

WORKDIR /sources/BoT-SORT/ 
ADD modified_files/bot_sort.py /sources/BoT-SORT/tracker/bot_sort.py
ADD modified_files/fast_reid_interfece.py /sources/BoT-SORT/fast_reid/fast_reid_interfece.py
RUN pip install .

RUN pip install open3d 
RUN pip install numpy==1.23.5

ADD communication_msgs/ /sources/catkin_ws/src/communication_msg
ADD openseed_src/openseed/body/encoder/ops/ /sources/catkin_ws/src/openseed_src/openseed/body/encoder/ops

RUN ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && \
    cd /sources/catkin_ws/src/openseed_src/openseed/body/encoder/ops && \
    ./make.sh"]
   

#git clone https://github.com/andrey1908/husky_tidy_bot_cv
ADD husky_tidy_bot_cv/ /sources/catkin_ws/src/husky_tidy_bot_cv/
ADD modified_files/bot_sort_node.py /sources/catkin_ws/src/husky_tidy_bot_cv/scripts/bot_sort_node.py


RUN ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && \
    cd /sources/catkin_ws/ && \
   /opt/ros/noetic/bin/catkin_make --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DPYTHON_EXECUTABLE=/usr/bin/python3.8 \
        -DPYTHON_INCLUDE_DIR=/usr/include/python3.8m \
        -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.8m.so"]

ADD openseed_src/ /sources/catkin_ws/src/openseed_src


RUN apt-get update && apt-get install -y git

# Clone the ultralytics repository
#RUN pip install ultralytics==8.0.135
#RUN git clone --branch v8.0.155 https://github.com/ultralytics/ultralytics.git /sources/catkin_ws/src/ultralytics

RUN git clone https://github.com/andrey1908/ultralytics.git /sources/catkin_ws/src/ultralytics
#RUN git clone https://github.com/ultralytics/ultralytics.git /sources/catkin_ws/src/ultralytics

# Install the ultralytics package
RUN pip install -e /sources/catkin_ws/src/ultralytics

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
CMD ["/bin/bash", "/entrypoint.sh"]
