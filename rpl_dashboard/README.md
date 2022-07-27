# Robotics Dashboard Web App for Laboratory Monitoring


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