## Extended Kalman Filter

[//]: # (Image References)

[dataset1]: ./etc/dataset1.png "Original dataset"
[dataset2]: ./etc/dataset2.png "Reversed dataset"
[lidar_only]: ./etc/lidar_only.png "Lidar only"
[radar_only]: ./etc/radar_only.png "Radar only"

In this project, we use an extended kalman filter to predict the location and velocity of a simulated bicycle that is traveling around the vehicle. The measurement data comes from lidar and radar sensors with the main algorithm is implemented in C++.

### Example of Predicted location
In the screenshots below, the green triangles represent the predicted location, the red circles are from the laser sensor, and the blue markers are from the radar sensor. We measure the accuracy of the algorithm by calculating the RMSE of the `x, y` positions and the velocity along the `x, y` axis. 

![alt text][dataset1]

* The original dataset starting with lidar measurement 

![alt text][dataset2]

* Reverse of the original dataset starting with radar measurement

If we just use one or the other of the sensor measurements to update the algorithm we can start to see what each sensor is better.

![alt text][lidar_only]

* The original dataset starting with lidar measurement and only using the lidar measurements to update to algorithm. We can see that compared to using both sources of sensor data the overall algorithm performs worse. 

![alt text][radar_only]

* The original dataset starting with lidar measurement and only using the radar measurements to update to algorithm. Compared to using only the lidar data, the radar only updated algorithm is worse at localizing the positon (higher RMSE for `x` and `y`).

### Compile and Build
In order to compile and build this project, make sure that the following dependencies are met.

* `cmake`:
  * For Mac make sure that `cmake` is at least version 3.5
* `make`:
  * For Mac make sure that `make` is at least version 4.1
* `gcc/g++`:
  * For Mac make sure that `gcc/g++` is at least version 5.4
* `uWebSocketIO`
  * From the project directory run `install-mac.sh`, which should be linked to the necessary `cmakepatch.txt` file
  * In order to run the above shell script, `homebrew` should also be installed

Once the above dependencies are installed:

1. Clone this repository
2. Create a build directory and navigate into it
  * `mkdir build && cd build`
3. Compile 
  * `cmake .. && make`
4. Run the program
  * Make sure that the [Udacity simulator](https://github.com/udacity/self-driving-car-sim/releases) is installed
  * `./ExtendedKF`


