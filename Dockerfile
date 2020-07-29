FROM ubuntu:18.04

RUN export LC_ALL=C.UTF-8
RUN export DEBIAN_FRONTEND=noninteractive
RUN ln -fs  /usr/share/zoneinfo/Europe/Rome /etc/localtime
RUN apt-get update && apt-get install -y tzdata
RUN dpkg-reconfigure --frontend noninteractive tzdata

RUN apt-get install -y\
   git\
   postgresql\
   postgresql-server-dev-all\
   libecpg-dev\
   automake\
   libtool\
   libcfitsio-dev\
   libnova-dev\
   libecpg-dev\
   gcc\
   g++\
   libncurses5-dev\
   libgraphicsmagick++1-dev\
   libx11-dev\
   docbook-xsl\
   xsltproc\
   libxml2-dev\
   libarchive-dev\
   libjson-glib-dev\
   libsoup2.4-dev\
   pkg-config\
   libwcstools-dev\
   liberfa-dev\
   libmodbus-dev\
   libboost-dev\
   libmsgpack-dev

RUN useradd -M  -s /bin/bash user
RUN export LIB_ECPG="-lecpg"
RUN export ECPG="/usr/bin/ecpg"
RUN export CPPFLAGS=" -I/usr/include/postgresql -I/usr/include/postgresql/10/server "
USER user