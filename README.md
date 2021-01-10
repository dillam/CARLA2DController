# CARLA2DController
2D autonomous vehicle controller that navigates a racetrack using preset waypoints while maintaining desired speed in the CARLA simulator. A video of the controller can be found at [this link](https://www.youtube.com/watch?v=rB24rA7mKSE). 

This controller uses longitudinal speed control with PID and Pure Pursuit lateral control. Below you can see the trajectory of the vehicle and the forward speed mapped out. The target values are plotted in orange and the recorded values are plotted in blue. 

![trajectory](/images/trajectory.png)
![forward speed](/images/forward_speed.png)

## Usage
Dependences for this project can be found in ```requirments.txt```. 

To run the controller, start CARLA at a 30hz fixed time-step on the Racetrack map. Then run ```module.py```.

To disable live plotting, open ```options.cfg``` and change to ```live_plotting = false```
