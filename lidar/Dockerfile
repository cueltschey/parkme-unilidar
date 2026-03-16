ARG OS_VERSION=24.04

FROM ubuntu:$OS_VERSION

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    cmake \
    make \
    gcc \
    g++ \
    pkg-config \
    libboost-all-dev \
    libpcl-dev \
    libyaml-cpp-dev

RUN mkdir -p /app

WORKDIR /app

COPY . .

RUN mkdir -p /app/build

WORKDIR /app/build

RUN cmake ../ && \
    make -j$(nproc) && \
    make install

WORKDIR /app

CMD [ "sh", "-c", "/usr/local/bin/parkme_unilidar /lidar.conf $ARGS" ]
