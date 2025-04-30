Lab 11: Real-World Localization
===============================

Objective
---------

The goal of this lab was to localize the real robot using the Bayes filter's update step only based on a full 360° sweep of ToF sensor readings.
Unlike in the simulation lab last time, no prediction step was performed due to noisy motion.


Marked Locations
----------------

The robot was placed at the following known poses:

1. (-3, -2, 0°)
2. (0, 3, 0°)
3. (5, -3, 0°)
4. (5, 3, 0°)

These correspond to:

1. (-0.914, -0.610, 0°)
2. (0.000,  0.914, 0°)
3. (1.524, -0.914, 0°)
4. (1.524,  0.914, 0°)

Observation Loop Implementation
-------------------------------

I implemented the `perform_observation_loop()` method using BLE communication and asynchronous notifications. 
The robot completed a controlled spin using PID orientation control, logged sensor readings, and transmitted them over BLE. I had to modify my lab 9 code.


Localization Results
--------------------

The table below shows the comparison between ground truth and estimated belief (maximum belief cell) after each scan.

.. list-table::
   :header-rows: 1
   :stub-columns: 1

   * - Ground Truth (ft)
     - Ground Truth (m)
     - Belief Estimate (m)
     - Belief Orientation (°)
     - Probability
   * - (-3, -2)
     - (-0.914, -0.610)
     - (-0.914, -0.610)
     - 10
     - 1.0
   * - (0, 3)
     - (0.000, 0.914)
     - (0.000, 0.914)
     - 10
     - 1.0
   * - (5, 3)
     - (1.524, 0.914)
     - (1.524, 0.610)
     - 10
     - 1.0
   * - (5, -3)
     - (1.524, -0.914)
     - (1.524, -0.914)
     - 10
     - 1.0

Plots of the belief distribution and scan data are shown below for each location.

.. image:: imgs/l11_1_plot.png
   :width: 70%
   :align: center

.. image:: imgs/l11_2_plot.png
   :width: 70%
   :align: center

.. image:: imgs/l11_3_plot.png
   :width: 70%
   :align: center

.. image:: imgs/l11_4_plot.png
   :width: 70%
   :align: center

Observations
------------

The robot localized within a grid cell in each pose, which was much fast than I thought it would. 

Reflection
----------

This lab really demonstrated how cool the bayes filter is in the real world. The bayes filter was successfully able to infer pose from sensor readings despite sensor noise, imperfect rotation, and discretization. 
It was able to maintain accuracy and robustness in localization.

Acknowledgments
---------------

Thank you to the Fast Robots Staff for providing files. I took heavy inspiration from my friend Aidan McNay. Thanks Aidan.

