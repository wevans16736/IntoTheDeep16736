FROM ubuntu:latest
LABEL authors="joshuajanni"

RUN apt-get update -y && apt-get upgrade -y
RUN apt install git-all -y
ENTRYPOINT ["top", "-b"]