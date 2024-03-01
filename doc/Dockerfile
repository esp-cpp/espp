# esp-docs doesn't currently support 3.12 (see https://github.com/espressif/esp-docs/issues/9)
FROM python:3.11

WORKDIR /usr/src/app

# update apt-get
RUN apt-get update

# install doxygen
RUN apt-get install -y doxygen

COPY requirements.txt ./

RUN pip install --no-cache-dir -r requirements.txt
