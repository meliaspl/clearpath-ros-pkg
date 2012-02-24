#!/bin/bash

if [ "$(id -u)" != "0" ]; then
  echo "This script must be run as root." 1>&2
  exit 1
fi

DEST=/etc/udev/rules.d
cp -v clearpath_base/udev/* $DEST
cp -v clearpath_sensors/udev/* $DEST

service udev restart
