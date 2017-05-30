# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.14, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.14 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.14.0.zip).
  * If you have MacOS and have [Homebrew](https://brew.sh/) installed you can just run the ./install-mac.sh script to install this.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt --with-openblas`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/CarND-MPC-Project/releases).



## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

### The Model

The kinematic model we use for this project is a simplification of dynamic model that ignores tire forces, gravity, and mass.The model describes the car state as [*x, y, &#936;, v, cte, e&#936;*]. The *x, y* are 2D cartesian coordinates of the car. *&#936;* is the orientation of the car heading to. *v* is car velocity, *cte* is cross-track error while *e&#936;* is the orientation error.

The car actuator inputs are [*&#948;, a*], where *&#948;* for steering angle and *a* for acceleration. With these two inputs, the kinematic model can be described with following equations:

![kinematicModel](images/model_equations.png)

Here *Lf* is the distance between the center of mass of the vehicle and the front wheels.

### Timestep Length and Elapsed Duration

The MPC prediction horizon *T* is the duration over which future predictions are made. It is determined by the number of timesteps in the horizon *N* and elapsed duration between timesteps *dt*. The *N* determines the number of variables optimized by MPC. It is also the major driver of computational cost. A longer *dt* could result in less frequent actuations, which makes it harder to accurately approximate a continuous reference trajectory. But a shorter *dt* requires larger *N* for a fixed prediction horizon, thus increases computational cost.

Since our goal is to drive the car in about 50 miles per hour on the track, which is about 22 meters per second. We limit the prediction horizon to be about 10 meters, which means our prediction horizon is about 0.5 seconds. Beyond that, the track can change enough that it won't make sense to predict any futher into the future. We start test our implementation with reference velocity from 20 miles per hour, and limit our searh of *N* from 10 - 16, *dt* from 0.025 - 0.075. We gradually increased reference velocity once we obtain a good result and adjust the values again for the testing velocity. Our final choices of *N* is 10, and *dt* is 0.5. These two values seem work well for reference velocity from 20 to 60 miles per hour. Athough this settings works when the car speed go to 80 miles per hour in simulator, it does show high swing on the second turn after the bridge. This show we need to use a small *T* and different *N* and *dt* for speed higher than 80 miles per hour. 

### Polynomial Fitting and MPC Preprocessing

A 3rd order polynomial is fitted to waypoints in car coordinates. The waypoints passed from simulator are in global coordinates. To transform to car coordinates, they are multiplied with a transform matrix. Following code shows the computation of transform matrix:

         Eigen::Matrix3d T;
          T << std::cos(psi), -std::sin(psi), px,
               std::sin(psi),  std::cos(psi), py,
               0,              0,             1;     

### Model Predictive Control with Latency

There is always a delay from MPC computation to the executation of actuation command. For this project, the latency is assumed to be 100 millisecond. To handle the latency, we estimate the car's prospective state based on its current speed and heading direction. The result is used as car's initial state for MPC trajectory computation. Following code show how the propective state is estimated:

          // taking account command's latency
          const double latency = 0.1;
          const double Lf = 2.67;
          const double cur_d = j[1]["steering_angle"];
          const double cur_a = j[1]["throttle"];

          double dx = v*std::cos(cur_d)*latency;  // car should have moved in x-direction for 100 millisec
          double dy = -v*std::sin(cur_d)*latency;
          double dpsi = -(v*cur_d*latency)/Lf;
          double dv = v + cur_a*latency;
          
          cte = polyeval(coeffs, dx);
          epsi = -CppAD::atan(coeffs(1)+coeffs(2)*dx+coeffs(3)*dx*dx);

          state << dx, dy, dpsi, dv, cte, epsi;
          vector<double> rc = mpc.Solve(state, coeffs);

Another approach to handle the latency is to fix the actuation values to previous values for the duration of latency. This approach requires the latency to be close multiple of *dt*. I believe the approach we take is more flexible. But the calulation of the initial position in current implementation is only approximate. For larger latency, we should use a more accurate formula.

## Result and Reflection

Here is a [link](https://www.youtube.com/watch?v=9Wah9d0_8SA) to my final video output:

<p align="center">
    <a href="https://www.youtube.com/watch?v=9Wah9d0_8SA">
        <img src="https://img.youtube.com/vi/9Wah9d0_8SA/0.jpg" alt="video output">
    </a>
</p>

We get much better result using MPC model compare to PID control. The only issue is when we go to very high speed (>80 miles/hour in this case), we'll see a big swing when there are two close turns. This may be due to the cost functions that require constant velocity. For very high speed, we may need a more flexible constraints on velocity and acceleration.
