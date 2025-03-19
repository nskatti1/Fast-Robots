====================================
Lab 6: Orientation Control
====================================

.. contents::
   :depth: 2
   :local:

Prelab
--------------------------------------------------------------------------
In this lab, I implemented orientation PID using the IMU. This required integrating the gyroscope to estimate orientation and tuning a PID controller for effective control; it very tedious, but I got it done. 

For communication, data was sent and received via Bluetooth. I wrote a command in C for my PID loop and decided I would put communications at the end of my PID loop and append it into an array and then end all that data back using a loop at the end.

.. code-block:: cpp

    float data_arr[6];

    data_arr[0] = millis();
    data_arr[1] = current_angle;
    data_arr[2] = target_angle;
    data_arr[3] = previous_error;
    data_arr[4] = control_signal;
    data_arr[5] = motor_offset;

    tx_estring_value.append(data_arr[0]);
    tx_estring_value.append(" | ");
    tx_estring_value.append(data_arr[1]);
     tx_estring_value.append(" | ");
    tx_estring_value.append(data_arr[2]);
     tx_estring_value.append(" | ");
    tx_estring_value.append(data_arr[3]);
    tx_estring_value.append(" | ");
    tx_estring_value.append(data_arr[4]);
    tx_estring_value.append(" | ");
    tx_estring_value.append(data_arr[5]);
    tx_estring_value.append(" | ");
    send_arr[i] = tx_estring_value.c_str();
    i=i+1;


Here is where I sent it back. I implemented this function within my STOP_PID function.

.. code-block:: cpp
    void SendStoredPIDData() {
        for (int i = 0; i < 500; i += 1) {
            tx_characteristic_string.clear();
            tx_characteristic_string.append(send_arr[i]);
            delay(100);
        }
    }

Lab
--------------------------------------------------------------------------

PID Controller Implementation and Testing
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The first step was implementing the PID controller to stabilize the robot's yaw using the IMU data. Below are the initial PID values I used:

- Kp = 7.65
- Ki = 1.53
- Kd = 20.4

Graphs of the angle versus time were generated to analyze the controller’s response.

.. list-table::
   :widths: auto
   :align: center

   * - .. image:: images/l6_graph1.jpg
          :width: 100%
     - .. image:: images/l6_graph2.jpg
          :width: 100%
     - .. image:: images/l6_graph3.jpg
          :width: 100%

Testing and Debugging
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Fine tuning the PID values took a lot of time and care and multiple tests. Here is what I observed:

- **Overshoot:** [Description]
- **Settling Time:** [Description]
- **Error Reduction Techniques:** [Description]

Here is are some videos of the robot stabilizing its orientation:

.. youtube:: [VideoID]
   :width: 560
   :height: 315

.. youtube:: [VideoID]
   :width: 560
   :height: 315

.. youtube:: [VideoID]
   :width: 560
   :height: 315


Lab Questions /Discussion
--------------------------
I answered some lab questions below? 

**Are there any problems that digital integration might lead to over time? Are there ways to minimize these problems?** 

Digital integration of the gyroscope data can lead to drift due to sensor noise and small errors accumulating over time. This is often referred to as yaw drift and can result in incorrect orientation estimates. This can be minimized by using a complementary filter or Kalman filter ( haha Lab 7 lol) to fuse the IMU data with other sensors and correct drift. Resetting the orientation based off of landmarks and data collected can be useful as well (yay I did this or using the DMP, which has a built in filter can be helpful.

**Does your sensor have any bias, and are there ways to fix this? How fast does your error grow as a result of this bias?**

It is common for gyroscopes have a constant bias that causes the error to grow linearly over time. If you measure it while it is stationary, you can subtract the constant bias. Additionally, the DMP can be useful.

**Are there limitations on the sensor itself to be aware of? What is the maximum rotational velocity that the gyroscope can read?**
**Is this sufficient for our applications, and is there a way to configure this parameter?**

Each IMU has a maximum detectable rotational velocity before saturation occurs. According to the documentation of the IMU we use, it has "a full scale range of ±250 dps, ±500 dps, ±1000 dps, and ±2000 dps". If the robot rotates faster than this limit, the sensor will clip values, leading to incorrect readings. Making sure within the code that it is not going faster that that, can prevent this.

**Does it make sense to take the derivative of an integrated signal?**
Since the gyroscope provides angular velocity, integration is used to obtain orientation. The derivative would just return the same original signal which is useless.

**Does changing your setpoint while the robot is running cause problems with your implementation of the PID controller?**
A sudden change in the setpoint can cause a sharp spike in the derivative term and this can destabilize the robot. Low-pass filters can help.

**Is a lowpass filter needed before your derivative term?**
Yes, it is common to use a low-pass filter on the gyroscope readings before computing the derivative term  as it reduces the effect of high-frequency noise potentially causing instability in the controller.

**Can you control the orientation while the robot is driving forward or backward? Why would this be useful ? (not required)**
I did not, but it will be useful in the future when it comes down to following a path as it can move forward and backwards and tune itself.

Reflection 
-----------------------------
This lab improved my understanding of PID tuning. I learned a lot about sensor bias and it was a fun but sometimes demoralizing. Here are some 

Acknowledgements
-----------------------------
Thank you to Anunth Ramaswami for lending me his robot when mine stopped working. I based my PID values similar to his before my robot broke and continued to do so after. My code worked much more differently than his and I scaled and adjusted his values accordingly. Thank you to Aidan McNay for sitting in the same room as me and working for moral support. Thanks to course staff for their guidance.
