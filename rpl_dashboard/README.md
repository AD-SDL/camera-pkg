# Robotics Dashboard Web App for Laboratory Monitoring

## Directory and File Information
* This project requires ROS 2 Foxy, Python 3.8.10, OpenCV, Flask, plotly.dash, numpy
* Folder `dashboard_ws2` is a ROS workspace, and the ROS package name for this workspace is `dashboard` which can be 
found under `dashboard_ws2/src/dashboard/`
* All the python modules that I wrote that run on the Intel NUC are under `dashboard_ws2/src/dashboard/dashboard/*`
* The Flask app is under this current parent directory and is named `dashboard_app.py`
* To build the ROS workspace and all its packages, run the bash script `dashboard_ws2/build_dashboard` as such: 
`source dasbhoard_ws2/build_dashboard`

## How to run application
First `cd` into `dashboard_ws2` directory and run 
`source ./build_dashboard`. You must run this anytime
any change is made to a python file inside the `dashboard_ws2` directory.


**Order in which programs need to be ran**  
_Note: Any commands w/ `ros2` should be ran from `dashboard_ws2` workspace folder. Command 2 to run the flask app should be ran from the current directory_
1. `ros2 run dashboard msg_pub`
2. `python3 dashboard_app.py`
3. `ros2 run dashboard cam_sub`
4. `ros2 run dashboard cam_pub`
5. `ros2 run dashboard msg_sub`