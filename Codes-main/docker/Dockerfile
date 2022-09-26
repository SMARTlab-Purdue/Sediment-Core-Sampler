FROM ros:melodic

RUN apt-get update \
  && apt-get install -y \
  nano \
  python3-pip \
  ros-melodic-rosserial \
  ros-melodic-usb-cam \
  ros-melodic-web-video-server \
  python3-rospkg-modules \
  && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt
