## UAV Simulator for PX4  ##


Try to make a light weight, standalone UAV simulator for PX4 firmware. It borrows the idea from JSBSim and [gazebo](https://github.com/PX4/sitl_gazebo).

### Todo

Initialize the model;
forces and moments calculation;
sensor message;
Mavlink communication with Pixhawk 


### References

[Stevens and Lewis, "Aircraft Control and Simulation",  Third Edition, 2016](http://www.wiley.com/WileyCDA/WileyTitle/productCd-1118870980.html)

[Randal W. Beard and Timothy W. McLain "Small Unmmaned Aircraft", 2012](http://press.princeton.edu/titles/9632.html)

[JSBSim](http://jsbsim.sourceforge.net/)

calculate lon and lat with spherical earth assumption.  Just for simulation.
  * http://www.movable-type.co.uk/scripts/latlong.html
  * http://www.movable-type.co.uk/scripts/latlong-vincenty.html
