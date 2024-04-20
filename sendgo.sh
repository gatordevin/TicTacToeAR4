#!/bin/bash

ros2 topic pub -1 /vis/update std_msgs/msg/String "{data: 'go!'}" 
