# novatel_span_driver

This ROS package connects via Ethernet to a [NovAtel](http://www.novatel.com/) receiver running
[SPAN](http://www.novatel.com/span).

Please see the ROS Wiki for details: http://wiki.ros.org/novatel_span_driver


## In This Fork

* Fixed covariance calculations
	* Original driver used 2^std_dev
	* Changed to correct std_dev^2
* Added RAW IMU publishing
* Changed the IMU publishing from INS
* Make sure that you set the correct scale and rates in the launch file


## Correct Bad Bags

* There is a python script that will convert bags that have incorrect covariances to the correct ones
* Please run `python bag_fix_gps.py <bag-1> .... <bag-n>`
* This will correct the 2^std_dev covariances into the std_dev^2 covariances

