
Lab 12: Self-Flippable Inverted Pendulum Car
====================================

.. contents::
   :local:
   :depth: 2

Introduction
------------

When figuring out what to do for the final project. My dear classmate, Anunth Ramaswami, implemented a controller inspired by Stephen Wagner's work the previous year. My original idea was to do the same and then get our cars to balance on top of each other, but my car would not cooperate, so we pivoted so that I would figure out how to make the car flip into the stunt. You can read  more about it below, but I essentially did this by implementing a state machine, and running trials to see the balance of how it could get into flip position. Then my classmate Aravind Ramaswami implemented a Kalman filter.

Our project was to take  on the challenge of developing a car that not only balances like an inverted pendulum but also flips up autonomously from a horizontal position to achieve that balance. This required integrating nonlinear system modeling, control theory, real-time estimation, and embedded systems engineering into one complete design. Together we had a blast.

Approach
--------

To achieve this behavior, we implemented the following steps:

1. **System Modeling**: Derive nonlinear equations using Lagrangian mechanics and linearize about an equilibrium point.
2. **Kalman Filtering**: Construct a discrete-time observer for estimating the system state.
3. **State-Space Control**: Use pole placement for state feedback stabilization.
4. **Flip Code**: Implement commands to physically flip the robot up.
5. **State Machine Integration**: Combine all components to operate in sync at the correct moments.

Realistically, the actual order was Anunth making the controller, I integrating the state machine so it could go into a wheelie , and then Aravind doing the math so he could implement a Kalman Filter for this system.

System Modeling
---------------

**Physical Model**
 
We modeled the car as a **wheel and inverted pendulum** system. The state vector:

.. math::

   \mathbf{x} = \begin{bmatrix} x \\ \dot{x} \\ \theta \\ \dot{\theta} \end{bmatrix}

Where :math:`x` is the axle position and :math:`\theta` is the angle of the pendulum from vertical.

**Energy Expressions**

Using Lagrangian mechanics:

.. math::

   T = \frac{1}{2} M \dot{x}^2 + \frac{1}{2} m (\dot{x}_{rod}^2 + \dot{y}_{rod}^2) + \frac{1}{2} I \dot{\theta}^2

.. math::

   V = -m g l \cos\theta

Lagrangian:

.. math::

   L = T - V

**Euler-Lagrange Equations**

We applied:

.. math::

   \frac{d}{dt}\left(\frac{\partial L}{\partial \dot{q}_i}\right) - \frac{\partial L}{\partial q_i} = Q_i

For :math:`x` and :math:`\theta`, yielding coupled nonlinear equations.

**Linearization**

We applied small-angle approximations:

.. math::

   \sin\theta \approx \theta, \quad \cos\theta \approx 1

Which allowed us to derive a linear state-space model:

.. math::

   \dot{\mathbf{x}} = A \mathbf{x} + B u

With:

.. math::

   A = \begin{bmatrix} 0 & 1 \\ \alpha_1 & 0 \end{bmatrix}, \quad
   B = \begin{bmatrix} 0 \\ -\alpha_2 \end{bmatrix}

Where:

.. math::

   \alpha_1 = \frac{(M + m)mgl}{D}, \quad \alpha_2 = \frac{ml}{rD}

**Controllability and Observability**

.. math::

   \mathcal{C} = [B, AB], \quad \mathcal{O} = \begin{bmatrix} C \\ CA \end{bmatrix}


By checking the ranks of the controllability and observability matrices, we verified we could place the poles of the closed-loop system anywhere in the complex plane in discrete time. This is critical when designing a system that must recover quickly from disturbances and avoid oscillation. Both were full-rank, so the system is controllable and observable.

Controller Design
-----------------

We used MATLAB's `place()` with poles at 0.87 and 0.75. This gave:

.. math::

   K = [0.04, 0.002]

The system was discretized using Euler method with dt = 0.017. Controller was implemented as:

.. code-block:: cpp

   float u = k_theta * theta + k_omega * omega;


Here is the code for the controller function:

 Controller Function

.. code-block:: cpp

   void controller(float reading, float desire, float om) {
     float kp = 0.04;
     float kd = 0.002;
     float e = reading - desire;
     float d_term = kd * om;
     float u = kp * e + d_term;

     int dir_r = -1, dir_l = -1;
     if (u < 0) {
       dir_r = 1;
       dir_l = 1;
     }

     float u_abs = abs(u);
     if (abs(e) > 70) {
       stop_motors();
       return;
     }

     command_motors(u_abs, u_abs, dir_r, dir_l, 30);
   }

Kalman Filter
-------------

We adapted the Kalman Filter from Lab 7 with updated A, B, C matrices. Process noise :math:`Q` was larger than measurement noise :math:`R` because we trusted the IMU more than the model.

The Kalman Filter allowed us to fuse two streams of sensor data: Angle from DMP(quaternion converted) and  Angular velocity from gyroscope. The angular velocity from gyroscope was fast but noisy and subject to bias and the angle from DMP was relatively smooth, but low-rate and could drift under dynamic conditions. The Kalman Filter was able to compensate for sensor limitations and provide reliable estimates of both angle and angular velocity, which fed into the controller.

.. code-block:: cpp

   void kalman_filter(float y1_rad, float y2_rad, float u_rad) {
     float y1 = y1_rad * 3.14159 / 180;
     float y2 = y2_rad * 3.14159 / 180;
     float u = u_rad;

     BLA::Matrix<2, 2> Ad = { ... };
     BLA::Matrix<2, 1> Bd = { ... };
     BLA::Matrix<2, 1> mu_p = Ad * mu + Bd * u;
     mu_p(1, 0) = -mu_p(1, 0);
     BLA::Matrix<2, 2> sigma_p = Ad * sigma * ~Ad + sigma_u;

     if (new_measurement == 1) {
       BLA::Matrix<2, 2> sigma_m = C * sigma_p * ~C + sigma_z;
       Invert(sigma_m);
       BLA::Matrix<2, 2> kkf_gain = sigma_p * (~C * sigma_m);
       mu = mu_p + kkf_gain * (BLA::Matrix<2, 1>{ y1, y2 } - C * mu_p);
       sigma = (I - kkf_gain * C) * sigma_p;
       new_measurement = 0;
     } else {
       mu = mu_p;
       sigma = sigma_p;
     }

     mu(0, 0) *= 180 / 3.14159;
     mu(1, 0) *= 180 / 3.14159;
   }


Flip State Machine
------------------

We observed that the controller only activates well past :math:`30^\circ`. Therefore, an open-loop sequence was implemented:

1. **FORWARD** — 272 ms
2. **BREAK** — 100 ms
3. **REVERSE** — 270 ms
4. **STOP** — wait for controller handoff

If the angle exceeds 30°, the controller and filter activate.

Before we even added the check for 30 degrees, I wrote a function `DELAY_STOP`. It is not named the best, but it was called that because that because the first function I implemented made the car go for a certain length of delay, and then it would abruptly stop. This did not make the car flip; it just made it go forward and stop(go figure). So I implemented it going forward and then suddenly reversing. This made it drift beautifully. Sometimes it went 360 degrees and continued. 

This is a blooper of it going a little more than 360 degrees, but I wanted to post it anyways because I thought it was cool

https://youtube.com/shorts/dXLb_GY04mo

Afterwards, we decided to try breaking the motors by supplying a pwm of 255 to each pin in between going forwards and backwards so it would coast before going in reverse. It successfully flipped. Here is a video.

https://youtube.com/shorts/OkugFH8zUUg

**This is NOT what I wanted**

If it flips, and lands back in its position, the controller would think that it is far from the target angle and then supply a large PWM signal. We had to write code that made it untrigger the controller if it detected that the  car was flat after the flip. 

Anyways now I needed to fine tune the values of how long it would be going forward and how long it would be going backwards. If I gave it too much acceleration for too long, it would flip over, and if I didn't give it enough time to go forward or reverse, the car wouldn't go up. 

**My goal was to make the car go up**. 

I eventually found that 272 ms for forward and 270 ms for backwards was perfect. Here is the code of the original sequence.

.. code-block:: cpp


    case DELAY_STOP:
    {

        success = robot_cmd.get_next_value(delay_val);
         if (!success)
           return;

       success = robot_cmd.get_next_value(stop_val);
         if (!success)
           return;
       command_motors(1,1, 1,1, 90);

       delay(delay_val);

       break_motors();

       delay(100);

       command_motors(1,1, -1,-1, 90);
      
       delay(stop_val);
       break_motors();
       delay(100);
       stop_motors();


      break;

    }

But now we wanted to implement this into Anunth's code because his file had the controller implemented as function with flags in the main loop. 
Because of this, I rewrote the code and turned `DELAY_STOP` into a flag and constant setter function as you can see below.

.. code-block:: cpp

    case DELAY_STOP:
    {
      success = robot_cmd.get_next_value(delay_val);
        if (!success)
          return;

      success = robot_cmd.get_next_value(stop_val);
        if (!success)
          return;
      flip_active = true;
      flip0 = true;
      flip1 = false;
      flip2 = false;
      flip3 = false;
      flip_start_time = millis();


      break;

    }

These flags are used in the state machine below.


.. code-block:: cpp

   if (flip_active){
        IMU_DMP_Yaw(); 
        if(abs(DCM_yaw[w-1])<60){
          start_O_controller = true;
          start_IMU = true;
          flip_active = false;
          mu(0,0) = DCM_yaw[w-1];
          mu(1,0) = -omega[w-1];
          //Serial.println("Controller Activated");
        }
        if (flip0){
          command_motors(1, 1, 1, 1, 90); 
          u_O[w-1] = 1;
          //Serial.println("state0");
          if (millis() - flip_start_time >= delay_val) {
            flip0 = false;
            flip1 = true;
            flip_start_time = millis();
            //Serial.println("transition");
          }
        }

        if (flip1){
          //Serial.println("state1");
          break_motors();
          u_O[w-1] = 0;
          if (millis() - flip_start_time >= 100) {
            flip1 = false;
            flip2 = true;
            flip_start_time = millis();
            //Serial.println("transition");
          }

        }

        if (flip2){
          //Serial.println("state2");
          command_motors(1, 1, -1, -1, 90);
          u_O[w-1] = -1;
          if (millis() - flip_start_time >= stop_val) {
            flip2 = false;
            flip3 = true;
            //Serial.println("transition");
          }
        }

        if (flip3){
          stop_motors();
          //Serial.println("state 3");
          flip3 = false;
        }

      }



Here is a diagram to make it easier to illustrate.

Controller and Kalman Filter Integration
----------------------------------------

Once the car has flipped up past a certain angle (approximately 30°), the system engages closed-loop control. This control process uses the Kalman filter to estimate the state (angle and angular velocity) and P controller to stabilize the system.

.. code-block:: cpp

   if (start_IMU && start_O_controller) {
     if (IMU_DMP_Yaw() == 0) {
       new_measurement = 1;
       kalman_filter(DCM_yaw[w - 1], -omega[w - 1], u_O[w - 1]);
       controller(mu(0, 0), 0, -mu(1, 0));
       KF_vals[w - 1] = mu(0, 0);
     }
   }

Activation occurs when:

.. code-block:: cpp

   if (abs(DCM_yaw[w - 1]) < 60) {
     start_O_controller = true;
     start_IMU = true;
     flip_active = false;
     mu(0, 0) = DCM_yaw[w - 1];
     mu(1, 0) = -omega[w - 1];
   }


Results
-------

.. figure:: fast_robots_final_plots/orientation_t2.png
   :align: center
   :figwidth: 70%



.. figure:: fast_robots_final_plots/u_sig_t2.png
   :align: center
   :figwidth: 70%



.. figure:: fast_robots_final_plots/orientation_t3.png
   :align: center
   :figwidth: 70%



.. figure:: fast_robots_final_plots/u_sig_t3.png
   :align: center
   :figwidth: 70%


.. youtube:: FdCVPBQw5X0
.. youtube:: WKfhfwsL8mU
.. youtube:: QNDRmvV0Qqg
.. youtube:: 5S5q_3baU6M
.. youtube:: QAAuYinvvWo
.. youtube:: szbKXjP3W68

Contributions
-------------

Anunth implemented the controller, Aravind implement the Kalman filter, and I implemented the flip/state machine. We all tested together and worked on integrating the code and getting the logging system in place.

Conclusion
----------

This lab was a great way to bring together many concepts from the semester, modeling, control, estimation, and real-time programming into one creative robotics stunt. We're proud to have achieved a self-flipping, self-balancing inverted pendulum car!

This project offered a chance to blend theory and practice. We derived the equations of motion from first principles, implemented estimation and control in real-time on embedded hardware, and we tuned, tested, and debugged in a physical environment subject to noise, delays, and imperfect actuation. This is project represented a compelling demonstration of applying classroom concepts, and it was cool to finish off the semester with this. Balancing an inverted pendulum is a classic control problem because it involves stabilizing an unstable equilibrium point. In our case, the pendulum (the car’s chassis) starts flat on the ground and needs to flip up into a vertical pose before any feedback controller can even operate. While a PID controller sufficed for balancing a pendulum with access to accurate state measurements, we decided to implement  state estimation via a Kalman Filter due to noisy sensor readings and the lack of reliable angular velocity from just the DMP to make our design even better.


Reflection
-----------

This was a cool project. It was a cool class. I will miss Cornell a lot. 

Shout out the Professor for being awesome and the TAs who were just as magnificent. To any future students of 4160, good luck - hope you enjoy the class as much as I did :)!!!!!!

