# CarND-Controls-PID
This project is done as a part of the Nanodegree - Self-Driving Car Engineer provided by Udacity. The scope of this project is the implementation of a PID controller, which allows a car (in a simulator) to follow the given track by adjusting the steering angle.

---
## PID Controller
A PID controller is used to minimize an error in the behavior of a technical system. In the case of a self-driving car, this error could be, for example, the distance between the car's actual position and the position where it should be. This error is called Cross Track Error (CTE).

A PID controller (Proportional-Integral-Derivative-Controller) consists of three parts and each of them must fulfill a certain measure, so that the entire system works properly.

**The following picture (taken from [UDACITY](https://www.udacity.com/) shows the influence of the single parts P, I and D):**

![PID](https://github.com/gada1982/CarND-PID-Control-Project/blob/master/data_for_readme/PID%20-%20Udacity.png)

### Proportional (P) Part of the Controller
The proportional part of the controller produces an output value that is directly proportional to the current error. This means the greater the deviation, the stronger is counter-steered. The result can be set by multiplying the error with a constant Kp (the proportional gain). The larger Kp is selected, the faster and stronger the controller reacts, but the more the output oscillates because of overshooting.

**The following picture (taken from [WIKIPEDIA](https://en.wikipedia.org/wiki/PID_controller) shows the influence of Kp):**

![P](https://github.com/gada1982/CarND-PID-Control-Project/blob/master/data_for_readme/P%20-%20Wiki.png)

### Integral (I) Part of the Controller
The integral part of the controller generates an output value by taking the sum of the instantaneous error over time. The integrated part is necessary to eliminate a possible systematic error of the technical system, which causes a permanent difference between the setpoint value and the current vehicle position. The result can be adjusted by multiplying the error with a constant Ki (the integral gain). The larger Ki is selected, the more the system is driven to the correct setpoint, but the system tends to overshoot.

**The following picture (taken from [WIKIPEDIA](https://en.wikipedia.org/wiki/PID_controller) shows the influence of Ki):**

![I](https://github.com/gada1982/CarND-PID-Control-Project/blob/master/data_for_readme/I%20-%20Wiki.png)

### Derivative (D) Part of the Controller
The derivative part of the controller produces an output value by determining the slope of the error over time. The result can be adjusted by multiplying the error with a constant Kd (the derivative gain). The larger Kd is selected, the more the oscillating power of the P-part is attenuated.

**The following picture (taken from [WIKIPEDIA](https://en.wikipedia.org/wiki/PID_controller) shows the influence of Kd):**

![D](https://github.com/gada1982/CarND-PID-Control-Project/blob/master/data_for_readme/D%20-%20Wiki.png)

## Implementation Details

### Error Calculation
The single errors of the PID controller have been calculated the following way:

**Proportional Part**

`p_error = cte`

**Derivative Part**

`d_error = cte - prev_cte`

**Integral Part**

`i_error += cte`

**Total Error**

The total error has been calculated the following way: 

`steer = -Kp*p_error - Kd*d_error - Ki*i_error;`

## Reflection

### Choosing the Hyperparameters
The design of the PID controller was developed the following way: First, a model of the controller was created in Excel, which was fed with test data (after observing the simulator's output). On this basis, a basic design could be determined which reacts as quickly as possible (Kp), does not oscillate too much (Kd) and at the same time corrects systematic offsets without adding to much oscillation effects to the system (Ki).

This basis was used in the simulator and was gradually, manually optimized. Kp was increased step by step to obtain a sufficiently fast response for narrow curves. When the system oscillated too much Kd was increased. Ki was chosen as small as possible without losing the correction of a systematic error.

The system behavior was comprehensible and corresponded to what was to be expected from the theoretical derivation on the basis of the model.

**Final Hyperparameters**

`Kp = 0.11`, `Ki = 0.0008` and `Kd = 2.5`

An already built solution with the useage of twiddle for parameter tuning was not used, because the different driving situations in the simulator varied too strongly to get a reliable result. Anyway, this gave some hints for manual parameter tuning.

## Simulation
The following [video](https://youtu.be/PULLG0IxSUU) shows a car, controlled by the implemented PID controller, successfully driving on a given track.

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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
