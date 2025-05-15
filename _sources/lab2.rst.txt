====================================
Lab 2: Sensor Fusion and High-Speed Data Sampling
====================================

.. contents::
    :depth: 2
    :local:

Overview
--------------------------------------------------------------------------
In this lab, we integrated data from an Inertial Measurement Unit (IMU) to estimate orientation by fusing accelerometer and gyroscope readings. Techniques such as noise analysis via FFT, low-pass filtering, and complementary filtering were used to refine the sensor data. We also optimized our code to achieve a high sampling rate, enabling robust real-time data acquisition.

Task 1: Initial Setup
--------------------------------------------------------------------------
To begin, the necessary libraries for the IMU were installed and the sensor was connected directly to the Artemis board. The IMU was first verified by running a basic tutorial that confirmed its ability to measure acceleration, angular rate, and magnetic fields. A simple blink sequence was added after initialization to provide a visual indicator of proper startup.

.. image:: insert_image_here.png
   :alt: IMU Setup Confirmation

Task 2: Accelerometer Data Processing
--------------------------------------------------------------------------
The accelerometer outputs data along the x, y, and z axes. In order to calculate pitch (``theta_a``) and roll (``phi_a``) from these values, calibration equations incorporating a correction factor (1.03) were applied—this factor was determined using a two-point calibration. Although the raw data was accurate, it contained significant high-frequency noise.

To analyze this noise, data was collected during controlled rotations and subjected to a Fast Fourier Transform (FFT). The FFT results indicated that the useful signal was mostly below 6 Hz, which led to the design of a low-pass filter using a recursive algorithm. With a sampling period of 10 ms and a cutoff frequency of 5 Hz, the filter coefficient (alpha) was computed to be 0.266.

Filtered accelerometer data for pitch and roll demonstrated a substantial reduction in high-frequency noise:

- **FFT Analysis:**

  .. image:: insert_image_here.png
     :alt: FFT Analysis of Accelerometer Data

- **Filtered Pitch:**

  .. image:: insert_image_here.png
     :alt: Filtered Pitch Data

- **Filtered Roll:**

  .. image:: insert_image_here.png
     :alt: Filtered Roll Data

Task 3: Gyroscope Integration and Fusion
--------------------------------------------------------------------------
Orientation was also computed by integrating the gyroscope’s angular rate measurements over time. While the gyroscope data was inherently smoother than the accelerometer readings, it suffered from drift over extended periods. To address this, a complementary filter was implemented to merge the gyroscope data with the filtered accelerometer values. After experimentation, a fusion parameter (gamma) of 0.8 was found to yield the most stable results.

Key results from this task include:

- **Gyroscope Drift:**

  .. image:: insert_image_here.png
     :alt: Gyroscope Drift in Pitch and Roll

- **Fused Orientation Data:**

  .. image:: insert_image_here.png
     :alt: Fused Sensor Data Combining Gyro and Accel

Task 4: High-Speed Data Sampling
--------------------------------------------------------------------------
For rapid data acquisition, all IMU functionalities were encapsulated in a single function, ``get_imu_data()``. This function is invoked in the main loop as soon as new data becomes available and a collection flag is set. By eliminating unnecessary delays, the system achieved a sampling rate of 367 Hz over a 5-second period.

Data is stored in three float arrays representing pitch, roll, and yaw, which offers a good compromise between precision and memory usage. Additional Bluetooth commands were implemented to start, stop, and transmit the collected data seamlessly.

.. image:: insert_image_here.png
   :alt: High-Speed Sampling Output

Robot Video
--------------------------------------------------------------------------
Below is a snapshot from a video showing the robot in action. The video demonstrates how the robust motors are capable of driving the robot—even to the point of flipping the car—highlighting the power and agility of the design.

.. image:: insert_image_here.png
   :alt: Robot Video Screenshot

Reflection
--------------------------------------------------------------------------
This lab provided valuable insights into sensor fusion and high-speed data acquisition. The process of combining noisy accelerometer data with drift-prone gyroscope measurements to produce a stable, fused orientation signal was both challenging and rewarding. The techniques learned here will be instrumental in future robotics projects where accurate and rapid sensor data processing is essential.

Acknowledgements
--------------------------------------------------------------------------
I would like to thank the lab TAs for their support throughout this project. Their insights and guidance were essential in overcoming the challenges encountered during the development of the sensor fusion and data sampling techniques.
