#!/bin/bash

# Store URL in a variable
URL1="https://docs.google.com/spreadsheets/d/1vy3FVGK3hEKARqIlJRl95kJv8RtJEy1JQcRPeJQpoEQ/edit#gid=0"
URL2="file:///home/pranjal/catkin_ws/src/pkg_task4/launch/index.html"

# Print some message
echo "** Opening $URL1 in Firefox **"

# Use firefox to open the URL in a new window
firefox -new-window $URL1 
firefox -new-window $URL2

