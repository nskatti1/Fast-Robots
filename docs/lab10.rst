Lab 10: Localization (sim)
====================================================

.. contents::
   :depth: 2
   :local:


Introduction
------------

In this lab, I implemented a Bayes Filter in simulation to accurately localize a robot’s pose within a discretized 3D grid, even in the presence of actuator and sensor noise. This foundational work provides the building blocks for future real-world localization tasks.

The Bayes Filter maintains a belief distribution over the robot’s possible states *(x, y, θ)*, which is updated at each time step through two key steps: the prediction step, and the update step. The prediction step uses odometry input and a motion model (with noise) to estimate where the robot likely moved. The update step refines this estimate by comparing actual range sensor readings to expected ones at each grid cell, reducing uncertainty.

These steps follow the Bayes update equations:

.. math::

   \overline{bel}(x_t) = \sum_{x_{t-1}} p(x_t \mid u_t, x_{t-1}) \cdot bel(x_{t-1})

.. math::

   bel(x_t) = \eta \cdot p(z_t \mid x_t) \cdot \overline{bel}(x_t)

At every iteration, the robot performs a 360° rotation to collect distance measurements, which are used to update the belief. The grid cell with the highest resulting belief represents the most probable pose of the robot at that time step.


Lab Tasks
---------

Implementation
--------------

### Compute Control

The `compute_control` function calculates the robot’s relative motion from one pose to the next. It returns `rotation1` (initial rotation), `translation` (straight-line distancee, and `rotation2` (final rotation).

.. code-block:: python

   def compute_control(cur_pose, prev_pose):
       """ Given the current and previous odometry poses, this function extracts
       the control information based on the odometry motion model.
   
       Args:
           cur_pose  ([Pose]): Current Pose
           prev_pose ([Pose]): Previous Pose 
   
       Returns:
           [delta_rot_1]: Rotation 1  (degrees)
           [delta_trans]: Translation (meters)
           [delta_rot_2]: Rotation 2  (degrees)
       """
       y_comp = cur_pose[1] - prev_pose[1]
       x_comp = cur_pose[0] - prev_pose[0]
       theta_prev = prev_pose[2]
       theta_cur = cur_pose[2]
       delta_1 = np.degrees(math.atan2(y_comp, x_comp)) - theta_prev
       delta_rot_1 = mapper.normalize_angle(delta_1)
       delta_trans = math.sqrt(y_comp**2 + x_comp**2)
       delta_rot_2 = mapper.normalize_angle(theta_cur - theta_prev - delta_1)
   
       return delta_rot_1, delta_trans, delta_rot_2

### Odometry Motion Model

The odometry motion model computes the probability that the robot transitioned from the  previous position the current position , given the control input u. It uses three independent Gaussian distributions to model noise in each motion component.

.. code-block:: python

   def odom_motion_model(cur_pose, prev_pose, u):
       """ Odometry Motion Model
   
       Args:
           cur_pose  ([Pose]): Current Pose
           prev_pose ([Pose]): Previous Pose
           (rot1, trans, rot2) (float, float, float): A tuple with control data in the format 
                                                      format (rot1, trans, rot2) with units (degrees, meters, degrees)
   
   
       Returns:
           prob [float]: Probability p(x'|x, u)
       """
   
   
       actual_u = compute_control(cur_pose, prev_pose)
       prob_rot_1 = loc.gaussian(actual_u[0] - u[0], 0, loc.odom_rot_sigma)
       prob_trans = loc.gaussian(actual_u[1] - u[1], 0, loc.odom_trans_sigma)
       prob_rot_2 = loc.gaussian(actual_u[2] - u[2], 0, loc.odom_rot_sigma)
       prob  = prob_rot_1 * prob_trans * prob_rot_2
       return prob


### Prediction Step

The prediction step loops over all prior grid cells with significant belief and spreads that belief across reachable cells using the odometry motion model. Beliefs are normalized afterward to prevent underflow.

.. code-block:: python


   def prediction_step(cur_odom, prev_odom):
       """ Prediction step of the Bayes Filter.
       Update the probabilities in loc.bel_bar based on loc.bel from the previous time step and the odometry motion model.
   
       Args:
           cur_odom  ([Pose]): Current Pose
           prev_odom ([Pose]): Previous Pose
       """
       u = compute_control(cur_odom, prev_odom)
   
       # Loop through all possible previous states
       u = compute_control(cur_odom, prev_odom)
       for ( x_idx, y_idx, a_idx ) in np.ndindex( loc.bel_bar.shape ):
         x_t = loc.mapper.from_map( x_idx, y_idx, a_idx )
         new_bel_bar = 0
         for ( x_idx_t_1, y_idx_t_1, a_idx_t_1 ), bel in np.ndenumerate( loc.bel ):
             if bel > 0.001:
                 x_t_1 = loc.mapper.from_map( x_idx_t_1, y_idx_t_1, a_idx_t_1 )
                 new_bel_bar += (
                     odom_motion_model( x_t, x_t_1, u ) *
                     bel
                 )
         loc.bel_bar[x_idx][y_idx][a_idx] = new_bel_bar

### Sensor Model

Each observation consists of 18 distance readings. For each grid cell, the expected readings are compared to the observed readings using a Gaussian likelihood function (per reading), assuming conditional independence.

.. code-block:: python

   def sensor_model(obs):
       """ This is the equivalent of p(z|x).
   
   
       Args:
           obs ([ndarray]): A 1D array consisting of the true observations for a specific robot pose in the map 
   
       Returns:
           [ndarray]: Returns a 1D array of size 18 (=loc.OBS_PER_CELL) with the likelihoods of each individual sensor measurement
       """
       prob_array = np.zeros(18)
       for i in range(18):
           prob_array[i] = loc.gaussian(loc.obs_range_data[i], obs[i], loc.sensor_sigma)
       
       return prob_array

### Update Step

The update step multiplies the predicted belief (`bel_bar`) by the sensor likelihood and normalizes the result.

.. code-block:: python

   def update_step():
       """ Update step of the Bayes Filter.
       Update the probabilities in loc.bel based on loc.bel_bar and the sensor model.
       """
    
       for x in range(mapper.MAX_CELLS_X):
           for y in range(mapper.MAX_CELLS_Y):
               for a in range(mapper.MAX_CELLS_A):
                   prob = np.prod(sensor_model(mapper.get_views(x,y,a)))
                   loc.bel[x,y,a] = loc.bel_bar[x,y,a] * prob
       loc.bel = loc.bel / np.sum(loc.bel)


Running the Filter
------------------

Each loop iteration performs the following:

.. code-block:: python

   for t in range(traj.total_time_steps):
       ...
       prediction_step(...)
       get_observation_data()
       update_step(...)

The belief is updated using motion and sensor data, and printed for debugging/visualization.

Simulation Results
------------------

Each run of the simulation shows, the ground truth trajectoy(green), the estimated trajectory from belief(blue), and the odometry only trajectory(red). The brighter the cell, the higher the belief is. Eventually the estimated belief converges to the ground truth after many iterations.

Here are photos from trial 1 and trial 2

 .. image:: images/l10_1.png
            :width: 30%

 .. image:: images/l10_2.png
            :width: 30%


Here are videos of trial 1 and trial 2

  .. youtube:: EiUtU0LOFPY
         :align: center


  .. youtube:: js9-ycH-V54
         :align: center


Tables
----------

Here is my table for trial 1. 

.. table:: Trial 1

    +------+--------------------------+--------------------------+-------------+--------------------------+---------------+
    | Step | GroundTruth              | BeliefState              | Probability | PosError                 | ErrYawWrapped |
    +======+==========================+==========================+=============+==========================+===============+
    | 0    | (0.277,-0.086,320.657)   | (0.305,0.000,-50.000)    | 1.0         | (-0.028,-0.086,370.657)  | 10.657        |
    +------+--------------------------+--------------------------+-------------+--------------------------+---------------+
    | 1    | (0.506,-0.521,657.739)   | (0.305,-0.610,-70.000)   | 1.0         | (0.201,0.089,727.739)    | 7.739         |
    +------+--------------------------+--------------------------+-------------+--------------------------+---------------+
    | 2    | (0.506,-0.521,994.819)   | (0.305,-0.610,-90.000)   | 1.0         | (0.201,0.089,1084.819)   | 4.819         |
    +------+--------------------------+--------------------------+-------------+--------------------------+---------------+
    | 3    | (0.539,-0.919,1354.819)  | (0.610,-0.914,-90.000)   | 1.0         | (-0.070,-0.005,1444.819) | 4.819         |
    +------+--------------------------+--------------------------+-------------+--------------------------+---------------+
    | 4    | (0.800,-1.057,1800.284)  | (0.914,-0.914,10.000)    | 1.0         | (-0.114,-0.143,1790.284) | -9.716        |
    +------+--------------------------+--------------------------+-------------+--------------------------+---------------+
    | 5    | (1.042,-1.271,2250.284)  | (0.914,-1.219,-10.000)   | 1.0         | (0.128,-0.052,2260.284)  | -0.284        |
    +------+--------------------------+--------------------------+-------------+--------------------------+---------------+
    | 6    | (1.210,-1.378,2700.284)  | (1.219,-1.219,-30.000)   | 1.0         | (-0.009,-0.159,2730.284) | -9.716        |
    +------+--------------------------+--------------------------+-------------+--------------------------+---------------+
    | 7    | (1.431,-1.431,3150.284)  | (1.524,-1.219,-50.000)   | 1.0         | (-0.093,-0.212,3200.284) | -0.284        |
    +------+--------------------------+--------------------------+-------------+--------------------------+---------------+
    | 8    | (1.621,-1.288,3600.284)  | (1.524,-0.914,-70.000)   | 1.0         | (0.097,-0.374,3670.284)  | -9.716        |
    +------+--------------------------+--------------------------+-------------+--------------------------+---------------+
    | 9    | (1.741,-1.047,4050.576)  | (1.829,-0.914,-90.000)   | 1.0         | (-0.088,-0.133,4140.576) | -0.576        |
    +------+--------------------------+--------------------------+-------------+--------------------------+---------------+
    | 10   | (1.748,-0.521,4500.828)  | (2.134,-0.610,-110.000)  | 1.0         | (-0.386,0.088,4610.828)  | -9.172        |
    +------+--------------------------+--------------------------+-------------+--------------------------+---------------+
    | 11   | (0.517,0.897,4570.828)   | (0.610,0.914,-110.000)   | 1.0         | (-0.093,-0.017,4680.828) | 0.828         |
    +------+--------------------------+--------------------------+-------------+--------------------------+---------------+
    | 12   | (0.307,0.294,4976.674)   | (0.305,0.305,-70.000)    | 1.0         | (0.002,-0.011,5046.674)  | 6.674         |
    +------+--------------------------+--------------------------+-------------+--------------------------+---------------+
    | 13   | (0.043,0.002,5267.931)   | (0.000,0.000,-130.000)   | 1.0         | (0.043,0.002,5397.931)   | -2.069        |
    +------+--------------------------+--------------------------+-------------+--------------------------+---------------+
    | 14   | (-0.333,-0.134,5605.390) | (-0.305,-0.305,-150.000) | 1.0         | (-0.028,0.171,5755.390)  | -4.610        |
    +------+--------------------------+--------------------------+-------------+--------------------------+---------------+
    | 15   | (-0.726,-0.111,5942.372) | (-0.610,0.000,-170.000)  | 0.942519    | (-0.116,-0.111,6112.372) | -7.628        |
    +------+--------------------------+--------------------------+-------------+--------------------------+---------------+


Here is my table for trial 2.

.. table:: Trial 2

    +------+--------------------------+--------------------------+-------------+--------------------------+---------------+
    | Step | GroundTruth              | BeliefState              | Probability | PosError                 | ErrYawWrapped |
    +======+==========================+==========================+=============+==========================+===============+
    | 0    | (0.272,-0.084,320.275)   | (0.305,0.000,-50.000)    | 1.0         | (-0.033,-0.084,370.275)  | 10.275        |
    +------+--------------------------+--------------------------+-------------+--------------------------+---------------+
    | 1    | (0.501,-0.519,657.739)   | (0.305,-0.610,-70.000)   | 1.0         | (0.196,0.090,727.739)    | 7.739         |
    +------+--------------------------+--------------------------+-------------+--------------------------+---------------+
    | 2    | (0.501,-0.519,995.583)   | (0.305,-0.610,-90.000)   | 1.0         | (0.196,0.090,1085.583)   | 5.583         |
    +------+--------------------------+--------------------------+-------------+--------------------------+---------------+
    | 3    | (0.540,-0.917,1355.583)  | (0.610,-0.914,-90.000)   | 1.0         | (-0.070,-0.003,1445.583) | 5.583         |
    +------+--------------------------+--------------------------+-------------+--------------------------+---------------+
    | 4    | (0.798,-1.050,1801.048)  | (0.914,-0.914,10.000)    | 1.0         | (-0.116,-0.135,1791.048) | -8.952        |
    +------+--------------------------+--------------------------+-------------+--------------------------+---------------+
    | 5    | (1.040,-1.258,2251.048)  | (0.914,-1.219,-10.000)   | 1.0         | (0.126,-0.039,2261.048)  | -1.048        |
    +------+--------------------------+--------------------------+-------------+--------------------------+---------------+
    | 6    | (1.208,-1.365,2701.048)  | (1.219,-1.219,-30.000)   | 1.0         | (-0.011,-0.146,2731.048) | -8.952        |
    +------+--------------------------+--------------------------+-------------+--------------------------+---------------+
    | 7    | (1.428,-1.418,3151.048)  | (1.524,-1.219,-50.000)   | 1.0         | (-0.096,-0.199,3201.048) | -1.048        |
    +------+--------------------------+--------------------------+-------------+--------------------------+---------------+
    | 8    | (1.617,-1.275,3601.048)  | (1.524,-0.914,-70.000)   | 1.0         | (0.093,-0.361,3671.048)  | -8.952        |
    +------+--------------------------+--------------------------+-------------+--------------------------+---------------+
    | 9    | (1.738,-1.033,4051.340)  | (1.829,-0.914,-90.000)   | 1.0         | (-0.091,-0.119,4141.340) | -1.340        |
    +------+--------------------------+--------------------------+-------------+--------------------------+---------------+
    | 10   | (1.744,-0.506,4501.592)  | (2.134,-0.610,-110.000)  | 1.0         | (-0.390,0.104,4611.592)  | -9.408        |
    +------+--------------------------+--------------------------+-------------+--------------------------+---------------+
    | 11   | (0.408,0.819,4575.059)   | (0.610,0.914,-110.000)   | 1.0         | (-0.202,-0.095,4685.059) | 5.059         |
    +------+--------------------------+--------------------------+-------------+--------------------------+---------------+
    | 12   | (0.240,0.191,4980.141)   | (0.305,0.305,-70.000)    | 1.0         | (-0.065,-0.114,5050.141) | 10.141        |
    +------+--------------------------+--------------------------+-------------+--------------------------+---------------+
    | 13   | (-0.003,-0.126,5272.544) | (0.000,-0.305,-130.000)  | 1.0         | (-0.003,0.178,5402.544)  | 2.544         |
    +------+--------------------------+--------------------------+-------------+--------------------------+---------------+
    | 14   | (-0.362,-0.289,5610.003) | (-0.305,-0.305,-150.000) | 1.0         | (-0.057,0.016,5760.003)  | 0.003         |
    +------+--------------------------+--------------------------+-------------+--------------------------+---------------+
    | 15   | (-0.748,-0.298,5947.080) | (-0.610,-0.305,-170.000) | 0.997736    | (-0.139,0.007,6117.080)  | -2.920        |
    +------+--------------------------+--------------------------+-------------+--------------------------+---------------+


Reflection
----------

The Bayes filter significantly improves localization performance compared to dead reckoning (odometry only). Somethings I noticed were that errors were lowest when the robot was near walls or corners (distinct sensor readings), and highest in open or symmetric spaces. Optimizations like skipping low-probability cells and vectorized operations allowed it to run in reasonable time.



Lab 10 References
-----------------

Thanks to the Fast Robots TAs, especially Mikayla Lahr whose webpage I took heavily inspiration from and constantly cross checked. I looked at Aravind Ramaswami's page for referencing his table and Aidan McNay's page odom_motion_model() function.
