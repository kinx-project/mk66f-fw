FROM debian:sid

RUN apt-get update && apt-get install -y \
    make \
    gcc-arm-none-eabi \
    && rm -rf /var/lib/apt/lists/*

VOLUME /usr/src/mk66f-fw
