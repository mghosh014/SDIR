FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive

RUN mkdir ~/sdir/

WORKDIR ~/sdir/

RUN mkdir ~/sdir/ctrl

RUN apt-get update && apt-get install -y build-essential cmake inetutils-ping

CMD ["sh", "/root/sdir/ctrl/build.sh"]

