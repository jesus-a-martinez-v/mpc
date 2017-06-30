
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
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from [here](https://github.com/coin-or/Ipopt/releases).
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


## Reflection

We were able to implement a Model Predictive Controller in charge of (safely) driving a car in a test track in a simulator. With a few reference values (these comprise our ideal state) and some hyperparameter tuning we did the trick! :)

### Reference values

We used three reference values:

  * ref_cte = 0: In our ideal world, the car should drive perfectly over the waypoints marked in the track, so by setting our reference cross-track error to 0 we are telling the MPC to strive for perfection!
  * ref_epsi = 0: Same reasoning behind the cross-track error.
  * ref_v = 120: By setting our reference velocity to 120 MPH we are telling the MPC to drive the car fast! Of course, this is for fun and experimentation purposes only. In a real life scenario velocity is a parameter that should be bound within a safe interval, depending on the driving conditions.

### Model

Our model is comprised of three main parts: State, Actuators and Equations. Let's start with the state.

#### State

This defines the situation of our car at some instant in time. The state is composed of:

  * Position in X axis (x).
  * Position in Y axis (y).
  * Car orientation (psi).
  * Velocity (v).
  * Cross-track error (cte): It measures how far is our car from the desired trajectory.
  * Orientation error (epsi): It measures how far is our car orientation from the desired one.

#### Actuators

These are the forces or influences that change the state. They are:

  * Steering angle (delta).
  * Acceleration (acceleration).

#### Equations

They describe how the *actuators* change our *state*. Here's the code that alters the state of each of our variables:

```
  void updateState(ADvector& fg, const ADvector& vars) {
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    for (unsigned short i = 0; i < N - 1; i++) {
      AD<double> x1 = vars[x_start + i + 1];
      AD<double> y1 = vars[y_start + i + 1];
      AD<double> psi1 = vars[psi_start + i + 1];
      AD<double> v1 = vars[v_start + i + 1];
      AD<double> cte1 = vars[cte_start + i + 1];
      AD<double> epsi1 = vars[epsi_start + i + 1];

      AD<double> x0 = vars[x_start + i];
      AD<double> y0 = vars[y_start + i];
      AD<double> psi0 = vars[psi_start + i];
      AD<double> v0 = vars[v_start + i];
      AD<double> cte0 = vars[cte_start + i];
      AD<double> epsi0 = vars[epsi_start + i];

      AD<double> delta0 = vars[delta_start + i];
      AD<double> a0 = vars[a_start + i];

      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 * x0;
      AD<double> psides0 = CppAD::atan(3 * coeffs[3] * x0 * x0 + 2 * coeffs[2] * x0 + coeffs[1]);

      fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[2 + psi_start + i] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
      fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
      fg[2 + cte_start + i] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[2 + epsi_start + i] = epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);
    }
  }
```

### Timestep Length and Elapsed Duration (N & dt)

The prediction horizon is the duration over which future predictions are made. We’ll refer to this as T. Knowing that T = N * dt, where N describes the number of points in the horizon and dt the time that passes between two consecutive actuations, we want to maximize T but striving to have a dt as small as possible. With this in mind we tried several different combinations of N and dt. Here are some of them:

  * N = 25, dt = 0.05: Makes the car wiggle a lot and abandon the track because the model.
  * N = 25, dt = 0.1: The car still wiggles (not as much as in the previous case) but still falls off track.
  * N = 25, dt = 0.3: The car drives "well" for a couple of meters but then falls off track. This is because the prediction horizon is set 7.5 seconds in the future. 

Finally, in order for the car to drive stable and focus only on the immediate future we decided that T must be equals to 1 second. For that matter, we set the final N to 10 and dt to 0.1.



### Polynomial Fitting and MPC Preprocessing

In order to facilitate our lives during the polynomial fitting stage and cte calculation, we shifted the car's reference frame to (0,0) and the heading angle to 0 degrees. Here's the code that does this:

```
for (unsigned short i = 0; i < ptsx.size(); i++) {
    double shift_x = ptsx[i] - px;
    double shift_y = ptsy[i] - py;

    ptsx[i] = shift_x * cos(0 - psi) - shift_y * sin(0 - psi);
    ptsy[i] = shift_x * sin(0 - psi) + shift_y * cos(0 - psi);
}

double* ptrx = &ptsx[0];
Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, 6);

double* ptry = &ptsy[0];
Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, 6);

auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);

double cte = polyeval(coeffs, 0);
double epsi = -atan(coeffs[1]);
```

Regarding the 100 milliseconds delay and how we handled, we resolved to project the state values that exact amount into the future before handing it to the MPC to do its job. Here's how:

```
double delay = 0.1;

double future_x = v * delay;
double future_y = 0;
double future_psi = -v * steer_value / Lf * delay;
double future_v = v + throttle_value * delay;
double future_cte = cte + v * sin(epsi) * delay;
double future_epsi = epsi - v * steer_value / Lf * delay;

Eigen::VectorXd state(6);
state << future_x, future_y, future_psi, future_v, future_cte, future_epsi;
```

### Cost functions:

We put most of the weight on the cross track error and on the orientations because our main goal was to maintain the car on track. We also put a fair amount of weight on the changes of steering angle (delta) and acceleration between consecutive points, so our cost function would be cautious of drastic changes that could make the car crash, leave the road or drive dangerously. Here are our cost functions in code:

```
  void updateCostUsingStateVariables(ADvector& fg, const ADvector& vars, double cte_weight, double angle_weight, double velocity_weight) {
    for(unsigned short i = 0; i < N; i++) {
       fg[0] += cte_weight * CppAD::pow(vars[cte_start + i] - ref_cte, 2);
       fg[0] += angle_weight * CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);
       fg[0] += velocity_weight * CppAD::pow(vars[v_start + i] - ref_v, 2);
     }
   }

  void updateCostUsingActuatorVariables(ADvector& fg, const ADvector& vars, double delta_weight, double acceleration_weight) {
    for(unsigned short i = 0; i < N - 1; i++) {
      fg[0] += delta_weight * CppAD::pow(vars[delta_start + i], 2);
      fg[0] += acceleration_weight * CppAD::pow(vars[a_start + i], 2);
    }
  }

  void updateCostUsingActuatorsChangeRate(ADvector& fg, const ADvector& vars, double delta_change_weight, double acceleration_change_weight) {
    for(unsigned short i = 0; i < N - 2; i++) {
      fg[0] += delta_change_weight * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
      fg[0] += acceleration_change_weight * CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
    }
  }
```

And our weights:

```
double cte_weight = 4000.0;
double angle_weight = 2000.0;
double velocity_weight = 1.0;

double delta_weight = 2.0;
double acceleration_weight = 3.0;

double delta_change_weight = 200.0;
double acceleration_change_weight = 10.0;
```