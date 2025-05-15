
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


System Modeling and Dynamics
------------------------------------

We represent the inverted RC car as a single rolling wheel of mass M with a rigid rod (the chassis) of mass m and length l attached at the axle. The wheel translates along the horizontal x‑axis without slipping, while the rod pivots about the axle.  Our state includes the position and angle of the system, and the control input \(u\) is the motor torque \(\tau\).

.. math::
  :nowrap:
   \mathbf{q} = \begin{bmatrix}
     x \\ \dot{x} \\ \theta \\ \dot{\theta}
   \end{bmatrix}, \quad u = \tau
We also introduce the following parameters:

* \(x\): horizontal coordinate of the wheel axle  
* \(\theta\): rod angle from vertical (counterclockwise positive)  
* \(l\): distance from axle to the rod’s center of mass  
* \(I\): moment of inertia of the rod about its center  
* \(M\): combined mass of the wheel and motor assembly  
* \(m\): mass of the rod plus any front-wheel mass  

The derivation that follows uses Lagrangian mechanics to obtain the equations of motion.

Geometry and Velocities
^^^^^^^^^^

The center of mass of the rod has coordinates:

.. math::

   \begin{aligned}
     x_{\mathrm{rod}} &= x + l \sin\theta,\\
     y_{\mathrm{rod}} &= -\,l \cos\theta
   \end{aligned}

Differentiating with respect to time gives:

.. math::
   :nowrap:

   \begin{aligned}
     \dot{x}_{\mathrm{rod}}
       &= \dot{x} + l \cos\theta\,\dot{\theta},\\
     \dot{y}_{\mathrm{rod}}
       &= l \sin\theta\,\dot{\theta}.
   \end{aligned}

Kinetic and Potential Energy
^^^^^^^^^^

The wheel’s kinetic energy is

.. math::

   T_{\mathrm{wheel}} = \tfrac12\,M\,\dot{x}^{2}

The rod’s kinetic energy comprises its translational and rotational parts:

.. math::

   T_{\mathrm{rod}}
   = \tfrac12\,m\bigl(\dot{x}_{\mathrm{rod}}^{2} + \dot{y}_{\mathrm{rod}}^{2}\bigr)
     + \tfrac12\,I\,\dot{\theta}^{2}

Substituting the expressions above yields:

.. math::

   T_{\mathrm{rod}}
   = \tfrac12\,m\bigl[(\dot{x} + l\cos\theta\,\dot{\theta})^{2}
     + (l\sin\theta\,\dot{\theta})^{2}\bigr]
     + \tfrac12\,I\,\dot{\theta}^{2}

Combining wheel and rod energies gives the total kinetic energy:

.. math::

   T = \tfrac12\,(M + m)\,\dot{x}^{2}
     + m\,l\,\cos\theta\,\dot{x}\,\dot{\theta}
     + \tfrac12\,(m\,l^{2} + I)\,\dot{\theta}^{2}

The potential energy of the rod (taking zero at axle height) is

.. math::

   V = -\,m\,g\,l\,\cos\theta

Lagrangian
^^^^^^^^^^

The Lagrangian \(\mathcal{L} = T - V\) becomes

.. math::

   \mathcal{L}
   = \tfrac12\,(M + m)\,\dot{x}^{2}
     + m\,l\,\cos\theta\,\dot{x}\,\dot{\theta}
     + \tfrac12\,(m\,l^{2} + I)\,\dot{\theta}^{2}
     + m\,g\,l\,\cos\theta

Euler–Lagrange Equations
^^^^^^^^^^

The general form is

.. math::

   \frac{d}{dt}\!\Bigl(\frac{\partial\mathcal{L}}{\partial\dot{q}_{i}}\Bigr)
   - \frac{\partial\mathcal{L}}{\partial q_{i}}
   = Q_{i}

Here \(q_{i}\in\{x,\theta\}\) and the generalized forces are \(Q_{x}=\tau/r\), \(Q_{\theta}=0\).

For \(x\):

.. math::

   \frac{d}{dt}\!\Bigl(\frac{\partial\mathcal{L}}{\partial\dot{x}}\Bigr)
   - \frac{\partial\mathcal{L}}{\partial x}
   = \frac{\tau}{r}
   \;\Rightarrow\;
   (M + m)\,\ddot{x}
   + m\,l\,\cos\theta\,\ddot{\theta}
   - m\,l\,\sin\theta\,\dot{\theta}^{2}
   = \frac{\tau}{r}

For \(\theta\):

.. math::

   \frac{d}{dt}\!\Bigl(\frac{\partial\mathcal{L}}{\partial\dot{\theta}}\Bigr)
   - \frac{\partial\mathcal{L}}{\partial \theta}
   = 0
   \;\Rightarrow\;
   (m\,l^{2} + I)\,\ddot{\theta}
   + m\,l\,\cos\theta\,\ddot{x}
   = m\,g\,l\,\sin\theta

Nonlinear Equations of Motion
^^^^^^^^^^
Combining the two gives

.. math::

   (M + m)\,\ddot{x} + m\,l\,\cos\theta\,\ddot{\theta}
   = \frac{\tau}{r} + m\,l\,\sin\theta\,\dot{\theta}^{2}

.. math::

   (m\,l^{2} + I)\,\ddot{\theta} + m\,l\,\cos\theta\,\ddot{x}
   = m\,g\,l\,\sin\theta

Linearization About the Upright Position
^^^^^^^^^^

For small \(\theta\) we approximate \(\sin\theta\approx\theta\), \(\cos\theta\approx1\), and neglect \(\dot{\theta}^{2}\).  The linearized form is

.. math::

   (M + m)\,\ddot{x} + m\,l\,\ddot{\theta} = \frac{\tau}{r}

.. math::

   (m\,l^{2} + I)\,\ddot{\theta} + m\,l\,\ddot{x} = m\,g\,l\,\theta

Solving these yields

.. math::

   \ddot{x}
   = \frac{1}{D}\Bigl((m\,l^{2} + I)\,\frac{\tau}{r}
     - m^{2}\,g\,l^{2}\,\theta\Bigr),
   \quad
   \ddot{\theta}
   = \frac{1}{D}\Bigl((M + m)\,m\,g\,l\,\theta
     - m\,l\,\frac{\tau}{r}\Bigr)

where

.. math::

   D = (M + m)\,(m\,l^{2} + I) - (m\,l)^{2}

Reduced \(\theta\) Dynamics
^^^^^^^^^^

Focusing on the pendulum alone:

.. math::

   \ddot{\theta}
   = \frac{(M + m)\,m\,g\,l}{D}\,\theta
     - \frac{m\,l}{r\,D}\,\tau

State‑Space Representation
^^^^^^^^^^

Define the reduced state

.. math::

   \mathbf{x}
   = \begin{bmatrix}\theta \\ \dot{\theta}\end{bmatrix},
   \quad
   u = \tau

so that

.. math::

   \dot{\mathbf{x}}
   = \begin{bmatrix}\dot{\theta} \\ \ddot{\theta}\end{bmatrix}
   = A\,\mathbf{x} + B\,u,
   \quad
   y = C\,\mathbf{x}

with

.. math::

   A = \begin{bmatrix}0 & 1 \\ \tfrac{(M + m)\,m\,g\,l}{D} & 0\end{bmatrix},
   \quad
   B = \begin{bmatrix}0 \\ -\tfrac{m\,l}{r\,D}\end{bmatrix},
   \quad
   C = \begin{bmatrix}1 & 0 \\ 0 & 1\end{bmatrix}

Controllability and Observability
^^^^^^^^^^

The controllability matrix is

.. math::

   \mathcal{C}
   = \bigl[\,B\;\;A\,B\bigr]
   = \begin{bmatrix}
       0 & -\tfrac{m\,l}{r\,D} \\
      -\tfrac{m\,l}{r\,D} & 0
     \end{bmatrix}

and the observability matrix is

.. math::

   \mathcal{O}
   = \begin{bmatrix}C \\ C\,A\end{bmatrix}
   = \begin{bmatrix}
       0 & 1 \\
       1 & 0 \\
       0 & 0 \\
       0 & 0
     \end{bmatrix}

Both have full rank (\(2\)), so the reduced system is controllable and observable.  We can therefore apply a Kalman filter to estimate \(\hat{\mathbf{x}}\) and a state‑feedback law

.. math::
   :nowrap:

   u = -K\,\hat{\mathbf{x}}

Discretization and Pole Placement
^^^^^^^^^^

Introduce

.. math::

   \alpha_{1} = \frac{(M + m)\,m\,g\,l}{D},
   \quad
   \alpha_{2} = \frac{m\,l}{r\,D}

so that

.. math::

   A = \begin{bmatrix}0 & 1 \\ \alpha_{1} & 0\end{bmatrix},
   \quad
   B = \begin{bmatrix}0 \\ -\alpha_{2}\end{bmatrix}

For a sampling period \(\Delta t\) and desired pole locations, MATLAB’s place() yields

.. math::

   K = \begin{bmatrix}2.29 & 0.34\end{bmatrix}

These gains assume \(\theta\) is in radians; multiply by \(\pi/180\) if your controller uses degrees.

Controller Implementation
-----------------

We used MATLAB's `place()` with poles at 0.87 and 0.75. This gave:

.. math::

   K = [0.04, 0.002]

The system was discretized using Euler method with dt = 0.017 because that was the average value we got between time stamps. Controller was implemented as:

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

The controller is very robust. Here is a video demonstration. 

.. youtube:: QNDRmvV0Qqg

Kalman Filter Implementation
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

.. youtube:: dXLb_GY04mo

Afterwards, we decided to try breaking the motors by supplying a pwm of 255 to each pin in between going forwards and backwards so it would coast before going in reverse. It successfully flipped. Here is a video.

.. youtube:: OkugFH8zUUg

**This is NOT what I wanted**

If it flips, and lands back in its position, the controller would think that it is far from the target angle and then supply a large PWM signal. We had to write code that made it untrigger the controller if it detected that the  car was flat after the flip. 

The code we added was this:

..code-block:: cpp

     if(abs(e)>70){
       stop_motors();
       return;
     }

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

   if (abs(DCM_yaw[w - 1]) < 60) {
     start_O_controller = true;
     start_IMU = true;
     flip_active = false;
     mu(0, 0) = DCM_yaw[w - 1];
     mu(1, 0) = -omega[w - 1];
   }

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


.. figure:: fast_robots_final_plots/statemachine.jpg
   :align: center
   :figwidth: 70%



Controller and Kalman Filter Integration
----------------------------------------

Once the car has flipped up past a certain angle (approximately 30°), the system engages closed-loop control. This control process uses the Kalman filter to estimate the state (angle and angular velocity) and P controller to stabilize the system. You can see that the functions implemented above are called when the flags are set.

.. code-block:: cpp

   if (start_IMU && start_O_controller) {
     if (IMU_DMP_Yaw() == 0) {
       new_measurement = 1;
       kalman_filter(DCM_yaw[w - 1], -omega[w - 1], u_O[w - 1]);
       controller(mu(0, 0), 0, -mu(1, 0));
       KF_vals[w - 1] = mu(0, 0);
     }
   }

Data Logging
-------------
As you can see in the State Machine, we appended the pwm signal values to an array and that we did the same in the controller as well with imu data.
We also appended values in from the imu and sent it back.


Results 
-------

**Example 1**

.. youtube:: FdCVPBQw5X0

.. figure:: fast_robots_final_plots/orientation_t2.png
   :align: center
   :figwidth: 70%



.. figure:: fast_robots_final_plots/u_sig_t2.png
   :align: center
   :figwidth: 70%



**Example 2**

.. youtube:: WKfhfwsL8mU

.. figure:: fast_robots_final_plots/orientation_t3.png
   :align: center
   :figwidth: 70%



.. figure:: fast_robots_final_plots/u_sig_t3.png
   :align: center
   :figwidth: 70%





Extra Videos 
-------------

More videos to show of it working. Note that in the first video, the `STOP_CONTROLLER` function that stops the controller was called, so the car falls over.

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

