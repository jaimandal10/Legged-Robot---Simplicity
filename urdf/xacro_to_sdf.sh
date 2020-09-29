#!/usr/bin/env bash
echo "Start"
rm -rf simplicity.sdf simplicity.urdf
rosrun xacro xacro.py simplicity.xacro > simplicity.urdf
gz sdf -p simplicity.urdf > simplicity.sdf
echo "DONE"
