#!/bin/bash

roscore & 
stdbuf -o L catkin test cdcpd -DCMAKE_EXPORT_COMPILE_COMMANDS=1
