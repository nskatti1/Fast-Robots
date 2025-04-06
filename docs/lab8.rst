====================================
Lab 8: Stunts!
====================================

.. contents::
   :depth: 2
   :local:

Objective
-----------------------------
The objective of this lab was to execute a high-speed robot stunt that combines everything we’ve built in Fast Robots up to this point — from motor control and sensing to Kalman filtering and PID tuning. 

I chose to execute the flip task, so I had to reinforce my robot batteries as previously I let them just sit on top of my robot and when trying to implement the lab, the batteries kept detaching.

.. image:: images/l8_robot_full.png
   :align: center
   :width: 60%
   :alt: Robot Ready

Setup and Preparation
-----------------------------

Before attempting the stunt, I ensured all components were working properly — including the Artemis board, TOF sensors, and motor drivers. I also reviewed and tested my Kalman Filter and PID code from previous labs.

I edited my kalman filter code from lab 7 to gather data on my car when I use a pwm signal of 250. I previously used a smaller signal in Lab 7 and wanted to get data on the car moving towards the Wall at the constant pace. Here is the data I got from that.

.. image:: images/l8_kf_dist.png
   :align: center
   :width: 60%
   :alt: Kalman Distance

.. image:: images/l8_kf_vel.png
   :align: center
   :width: 60%
   :alt: Kalman Velocity

Stunt Execution: Task A - Flip
-----------------------------

My robot began at the designated start line and accelerated forward. I tuned the flip trigger point based on TOF/KF estimation or hardcoded timing. I added weight to the front to help initiate the flip.

Here is a photo of my robot mid-stunt:

.. image:: images/l8_flip_midair.png
   :align: center
   :width: 60%
   :alt: Flip Midair

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


Tuning and Control Strategy
-----------------------------

To make the stunt successful, I had to fine-tune the following:

- Trigger timing for flip/turn: 
    - I created an open loop that senses the uses the kalman filter to estimate the wall and then flips.
- Orientation Control 
    - I implemented orientation control from lab 4 to orient itself after it flips 
- PID loop
    - I used a PID loop from lab 5 to then have the robot return to me and stop in front of me(the person who set the robot down when starting it(therefore returning it to its original position). 

I tried adding  extra mass, and it actually messed up my flipping, so I did not add any extra. 

Here is are snippets of the code that managed my stunt logic:


Optional: Bloopers and Extra Credit
-----------------------------

Here is a blooper reel of some of my more chaotic attempts(its a meme if you will):

.. youtube:: _____________
   :width: 560
   :height: 315

Reflection
-----------------------------

This was a fun culmination of the what we have done so far in fast robots. It was cool coming up with ideas of how to execute and I felt like previous ideas we have used, ie PID, Kalman, etc. logically popped into my head when thinking. Overall, super cool lab. 

Acknowledgements
-----------------------------
 
Thank you to the people's whose house I visited for letting me use your crash mat setup and letting me run my car around your house.
Thank you to Glue for being the best cat ever(see blooper reels).
Thanks to the class staff for your help. 
I used ChatGPT to help structure this write-up and debug my C code. It catches unclosed parathesis. Thanks Chat!

