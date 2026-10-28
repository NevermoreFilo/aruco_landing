# aruco_landing
Repository for CV's exam project regarding landing

# What to do first
Replace your "models" folder with the one in this repo (or copy the models you need). Also, copy gazebo-iris-gimbal.param in your ardupilot_gazebo/config folder

# How to run
You'll need three terminals:

1)Start gazebo: gz sim -v4 -r iris_runway.sdf

2)Go to ~/ardupilot/Tools/autotest$ (or equivalent) and run: ./sim_vehicle.py -D -v ArduCopter -f JSON -w --no-rebuild --add-param-file=$HOME/gz_ws/src/ardupilot_gazebo/config/gazebo-iris-gimbal.parm 

Run main.py
