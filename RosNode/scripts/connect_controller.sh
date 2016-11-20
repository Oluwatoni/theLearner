#!/bin/bash
echo "CONNECT THE CONTROLLER IN 3 seconds"
sudo rfcomm bind rfcomm0 20:15:05:20:20:65
sixad --start && sixad -r
echo "connected to the ps3 controller and microcontroller"
