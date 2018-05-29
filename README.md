# PID Controller
In this project you'll revisit the lake race track from the Behavioral Cloning Project. This time, however, you'll implement a PID controller in C++ to maneuver the vehicle around the track!

The simulator will provide you the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle.

[//]: #	"Image References"
[pid_01]: ./results/steer_value_pid_0.1_0.0_0.0.jpg
[pid_02]: ./results/steer_value_pid_0.3_0.0_0.0.jpg
[pid_03]: ./results/steer_value_pid_0.05_0.0_0.0.jpg
[pid_04]: ./results/steer_value_pid_0.05_0.0_3.0.jpg
[pid_05]: ./results/steer_value_pid_0.1_0.001_1.0.jpg



---

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## PID Controller

### Structure of this repository

The source includes two major directory:

- [`/src-`]()- Contains `json.hpp`,  `main.cpp`,  `PID.cpp`,  `PID.h`,  `Twiddle.cpp`, and  `Twiddle.h`.

  > **Note:** I implemented twiddle algorithm but I didn't use it to produce the results.

- [`/results`]()- Containing:

  - The steering results with different PID coefficients values ( `*.csv` ).
  - The diagrams of steering value changes (`*jpg`).

### Implementing the PID controller

I started with using a P controller which had the following coefficients to control the steering value:

```C++
// P controller
std::vector<double> p = {0.1, 0.00, 0.0}; // {tau_p, tau_i, tau_d}
```

with this controller the car moves in an oscillation path and the CTE increases more and more when time goes and finally the car crashed. The result is as follows:

| The car crashes after iteration #570 |
| :----------------------------------: |
|         ![alt text][pid_01]          |

and if you increase the `tau_p` from 0.1 to 0.3 it you will see the same behavior but with a faster oscillation. The result is as follows:

| The car crashes after iteration #260 |
| :----------------------------------: |
|         ![alt text][pid_02]          |

So, based on the results it's obvious that I should decrease the value so change it to 0.05 and the result is as follows:

| The car crashes after iteration #1280 |
| :-----------------------------------: |
|          ![alt text][pid_03]          |

The car moves with a less oscillation but still is slow to come back on the track after it turns left or right and finally it crashes. So, a P controller is not enough. Then I used a PD controller with following coefficients:

```c++
// PD controller
std::vector<double> p = {0.05, 0.00, 3.0}; // {tau_p, tau_i, tau_d}
```

And the result is as follows:

| The car doesn't oscillate and never crashes |
| :-----------------------------------------: |
|             ![alt text][pid_04]             |

In this case, the car doesn't oscillate anymore and never crashes and it is an improvement! However, if you look at the above diagram carefully after the car come back to middle of the road again it deviates to the right. And again PD controller bring it back to middle. And the reason is that when time goes more and more it accumulates the error because it has a bias! And a PD controller is unable to solve this problem. Hens, to have a smother driving experience I will use a PID controller which has the following values:

```c++
// PID controller
std::vector<double> p = {0.05, 0.001, 3.0}; // {tau_p, tau_i, tau_d}
```

The car is moving a little bit smother. I tried different values for PID controller and finally tho following PID controller gave me smother experience:

```c++
// PID controller
std::vector<double> p = {0.01, 0.001, 1.0}; // {tau_p, tau_i, tau_d}
```

And the result is as follows:

| The car never crashes |
| :-------------------: |
|  ![alt text][pid_05]  |

You can see how the car is moving with this new PID controller [here]().

### License

