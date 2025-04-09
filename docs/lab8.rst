Lab 8: Stunts!
====================================

.. contents::
   :depth: 2
   :local:

Objective
-----------------------------
The objective of this lab was to execute a high-speed robot stunt that combined different ideas that we have learned about in Fast Robots and I am happy to report that it was a fun successful lab.

I chose to execute the drift task(Task B), but I initially wanted to do the flip, so I had reinforced my car with tape particularly to reinforce the batteries. This meant it was also prepared for drifting. Here is a pic of my car.

.. image:: images/l8_robot_full.png
   :align: center
   :width: 60%
   :alt: Robot Ready

Setup and Preparation
-----------------------------

Before attempting the stunt, I ensured all components were working properly — including the Artemis board, TOF sensors, IMU, and motor drivers. I also reviewed and tested my Kalman Filter and PID code from previous labs.

I modified my Kalman Filter code from Lab 7 to gather data on my car while using a PWM signal of 250. In Lab 7 I had used a lower PWM signal, so I wanted to understand how my robot behaved at higher speeds approaching a wall. Below is the data I collected. I did NOT tune my filter very well. I am disclosing this honestly, but at the end of the day my code still excuted and it still used this kalman filter so, I am showing the work I did for this lab on my website. 

.. image:: images/l8_kf_dist.png
   :align: center
   :width: 60%
   :alt: Kalman Distance

.. image:: images/l8_kf_vel.png
   :align: center
   :width: 60%
   :alt: Kalman Velocity

Stunt Execution: Task B - Drift
-----------------------------

I had the robot start away from the wall, and I tuned the drift trigger point based on TOF/KF estimation to begin the 180-degree turn approximately 914mm (3 floor tiles) from the wall.

Here are three successful runs:

.. youtube:: _____________
   :width: 560
   :height: 315

.. youtube:: _____________
   :width: 560
   :height: 315

.. youtube:: _____________
   :width: 560
   :height: 315

And here are the plots of the TOF data, PWM signals, and Kalman Filter Data.

.. image:: images/l8_kf_dist.png
   :align: center
   :width: 60%
   :alt: Kalman Distance

.. image:: images/l8_kf_dist.png
   :align: center
   :width: 60%
   :alt: Kalman Distance

.. image:: images/l8_kf_dist.png
   :align: center
   :width: 60%
   :alt: Kalman Distance


Tuning and Control Strategy
-----------------------------

To make the stunt successful, I used the Kalman Filter to estimate when the robot was within 914mm of the wall, then initiated the 180-degree spin. I used two functions START_DRIFT and STOP_DRIFT.

Here are snippets of the code that handled my drift logic and control timing:


Reflection
-----------------------------


Acknowledgements
-----------------------------

Thanks to the friends who let me take over their hallway and crash mats.  
Shoutout to Glue, the best cat - didn't quite make it into the blooper reel, but we still love him.  
And thank you to the course staff for your help.
Also, thanks to ChatGPT for helping me debug C code and for catching syntax errors in my write-up. You over edited and started changing my text and I had to revert. 

