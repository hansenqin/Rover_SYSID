# Purpose
The purpose of this class is to obtain the lateral tire characteristic curve of a rover. The rover is to perform a various number of maneuvers, while a rosbag records the necessary messages. Here are the basic steps of system identification:
1. **Frequency synchronization**: Since the signals in different topics are sampled at different time, all the signals must be converted to the same sample rate before we can analyze it.  
2. **Lowpass**: Getting rid of high frquency noise in the signals so signals can look smoother
3. **Time alignment**: Messages generated from different machines in a closed net could have some time shift relative to each other, and they need to be corrected to in order to perform analysis. Currently this step is omitted in tis code

# Usage
First initialize an instance of the class:
```
sys_id = tire_curve_sysID_helper_class("velodyne_rover_sysid.bag", "velodyne_rover.json", auto_flag_on=1)
```

The constructor of the class takes at least two inputs:
1. string: name of the bag file
2. string: name of the json file used for configuration of the rover

The constructor also accepts arbitrary number of optional inputs that can directly set the member variables of the class, as shown in the example above.


To obtain the tire coefficients, run the following method:
```
[fLinearCoef, rLinearCoef, fNonlinearCoef, rNonlinearCoef, alphaf, alphar, F_ywf, F_ywr] = sys_id.get_tire_curve_coefficients()
```
Where this returns both the coefficients for linear model and the nonlinear model of the tire


To plot the linear tire model, run the following method:
```
sys_id.plot_linear_tire_curve(fLinearCoef, rLinearCoef, alphaf, alphar, F_ywf, F_ywr)
```
Where this generates a plot for the front tire and a plot for the back tire.

To perform low speed sys ID, run the following method:
```
[time delta_u] = sys_id.low_speed_sys_id()
```
Where this generates a scatter plot of delta u vs time and a plot of delta u vs u. Note that the trajectories with SLAM issues must be filter manually, and the time interval must be chosen.
