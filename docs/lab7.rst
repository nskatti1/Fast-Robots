====================================
Lab 7: Kalman Filter
====================================

.. contents::
   :depth: 2
   :local:


Lab
--------------------------------------------------------------------------

Estimating Drag and Momentum
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

I applied a constant PWM signal of 200 and recorded the TOF sensor data to estimate steady-state speed and rise time. I then fit an exponential model to the velocity curve to extract the drag coefficient and effective mass. I planed the car around 3 m away from a wall.

Here are the plots of distance, computed velocity, and PWM:

.. image:: images/l7_tof_data.png
   :align: center
   :width: 70%
   :alt: TOF Data

.. image:: images/l7_velocity.png
   :align: center
   :width: 70%
   :alt: Velocity Plot

.. image:: images/l7_pwm.png
   :align: center
   :width: 70%
   :alt: PWM

.. image:: images/l7_fitted.png
   :align: center
   :width: 70%
   :alt: Fit

Steady-state velocity: ** 3110 mm/s**  
Rise time (90%): **  1.123 s**  
Velocity at 90% rise time: **2799 mm/s**

Using this, I computed drag and mass as:

.. code-block:: python

   code

Drag to Mass Coefficient Ratio: ** 2.9×10 −7**

Kalman Filter Initialization (Python)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

I used the discretized state-space model as follows:

.. code-block:: python

  code

My initial state vector and covariances:

.. code-block:: python

  code

Here is a photo of my covariance tuning and KF matrix setup:

.. image:: images/l7_kf_params.png
   :align: center
   :width: 60%
   :alt: KF Parameters

Implementing and Testing KF in Jupyter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

I processed the collected lab data through the Kalman Filter, adjusting the process and sensor noise to improve accuracy.

.. image:: images/l7_kf_results_raw.png
   :align: center
   :width: 70%
   :alt: Raw KF Output

.. image:: images/l7_tuned_kf.png
   :align: center
   :width: 70%
   :alt: Tuned KF Output

Here's my KF code in Python:

.. code-block:: python

 coe

I initially had issues with my covariance matrices, but that was because I was calculating them wrong. 

Kalman Filter on the Robot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

I implemented the Kalman Filter on the Artemis using the `BasicLinearAlgebra` library. I updated my `run_pid_step` function from Lab 5 to integrate the filtered distance into PID control.

Here is a video of my robot setup:

.. image:: images/l7_robot_setup.png
   :align: center
   :width: 60%
   :alt: Robot Setup

Here is a video of the robot stopping smoothly with the Kalman Filter:

.. youtube:: _____________
   :width: 560
   :height: 315

Here is a plot of raw vs filtered sensor data:

.. image:: images/l7_kf_robot_plot.png
   :align: center
   :width: 70%
   :alt: KF Robot Plot

And here is a zoomed-in version:

.. image:: images/l7_kf_robot_zoomed.png
   :align: center
   :width: 70%
   :alt: KF Robot Zoomed


Reflection
-----------------------------
This lab was pretty fun. There were times when my kalman filter was broken and I did not know what I was doing wrong, but I figured it out. Kalman filters are cool. I thought my robot was broken during lab 6, but it started functioning again. I think my battery had died during lab 6 because everything seemed to work this time.

Acknowledgements
-----------------------------
Thank you to the course staff for their help and my fellow classmates for using ed discussion. It has been very helpful.
I used ChatGPT to help debug code and also create a barebones outline of this report before filling in all the details(it created the headers). It also helped debug my plots.
