#!/bin/bash

DIR=sketchbook
rosrun avr_bridge gen_avr.py config.yaml $DIR
cd $DIR/avr_ros
ln -s . avr_ros
