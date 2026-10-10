# aruco_landing
Repository for CV's exam project regarding landing

# How to run
You'll need three terminals:

1)Start gazebo: gz sim -v4 -r iris_runway.sdf

2)Go to ~/ardupilot/Tools/autotest$ (or equivalent) and run: ./sim_vehicle.py -D -v ArduCopter -f JSON --add-param-file=$HOME/gz_ws/src/ardupilot_gazebo/config/gazebo-iris-gimbal.parm --console --map

3) Run the required script with python3
