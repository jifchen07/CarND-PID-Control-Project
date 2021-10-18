# CarND-Controls-PID

The goal of this project is to implement a PID controller in C++ to drive the vehicle around simulated track. 

### Effects of the P, I, D components in implementation:

* **P**: The proportional component of the PID controller steers proportional to the vehicle's offset against the centerline (cross-track error) and tries to steer the car towards the center of the lane. However, if P component is used alone without I and D components, overshoot occurs and the vehicle's driving trajectory starts to oscillate and goes out of the road very easily.
* **I**: The integral component counteracts the bias in the controlled system that prevents the vehicle from reaching the centerline. It reduces the steady state error. In the simulation step up, this system bias is negligible.
* **D**: The differential component counteracts the proportional component's tendency to overshoot. It controls proportionally to the rate of change in cross-track error and helps the car reach the centerline more smoothly with less oscillation.

### Tuning of P, I, D parameters:

#### Manual tuning:

1. Initialize all P, I, D gains to 0. 

2. Tune P gain value until the car start to follow the road (especially follow the curves) but has  consistent oscillation.
3. Tune D gain value, while also tune P gain value around the value found in step 2, until the oscillation is gone and vehicle can still follow sharp turns.
4. Tune I gain value to reduce long term accumulated system error.

#### Twiddle algorithm automatic tuning:

Once the approximate PID parameters are determined after manual tuning. Twiddle algorithm is applied to fine tune the parameters to further enhance the controller performance. I set the controller performance evaluation criteria to be the average cross-track error across the first 1000 steps(large enough to cover both several straight lines and sharp turns). After twiddling, the vehicle have much less average cross-track error but at the same time it makes a lot of abrupt small steering changes and drives much less smoothly. For this reason, I kept using the initial values gained after manual tuning. The final parameters are: P = 0.18, I = 0.0001, D = 2

### Dependencies

* cmake >= 3.5

 * All OSes: [click here for installation instructions](https://cmake.org/install/)

* make >= 4.1(mac, linux), 3.81(Windows)

  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)

* gcc/g++ >= 5.4

  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

* [uWebSockets](https://github.com/uWebSockets/uWebSockets)

  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.

  * If you install from source, checkout to commit `e94b6e1`, i.e.

    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 