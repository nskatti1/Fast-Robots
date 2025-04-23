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

Each run of the simulation shows, the ground truth trajectoy(green), the estimated trajectory from belief(blue), and the odometry only trajectory(red. The brighter the cell, the higher the beleif is. Eventually the estimated belief converges to the ground truth after many iterations. 


.. grid:: 2
    :gutter: 2

    .. grid-item::
        .. youtube::
            :align: center

    .. grid-item::
        .. youtube:: 
            :align: center

.. grid:: 2
    :gutter: 2

    .. grid-item::
        .. image:: 
            :width: 100%

    .. grid-item::
        .. image::
            :width: 100%


### Run 1 Results

.. image:: images/l10_run1_final.png
   :align: center
   :width: 80%
   :alt: Run 1 Final Belief Plot

// - **Iterations:** 15  
// - **Final Most Likely Pose:** (x ≈ 2.13, y ≈ 0.91, θ ≈ 60°)  
// - **Ground Truth Pose:** (x ≈ 2.20, y ≈ 0.88, θ ≈ 55°)

### Run 2 Results

.. image:: images/l10_run2_final.png
   :align: center
   :width: 80%
   :alt: Run 2 Final Belief Plot

// - **Iterations:** 15  
// - **Localization was robust despite early odometry drift.**  
// - Final belief track aligned well with the true path.

Tables
----------


Reflection
----------

The Bayes filter significantly improves localization performance compared to dead reckoning (odometry only). Somethings I noticed were that errors were lowest when the robot was near walls or corners (distinct sensor readings), and highest in open or symmetric spaces. Optimizations like skipping low-probability cells and vectorized operations allowed it to run in reasonable time.



Lab 10 References
-----------------

Thanks to the Fast Robots TAs, especially Mikayla Lahr whose webpage I took heavily inspiration from and constantly cross checked. I looked at Aravind Ramaswami's page for referencing his table. And Aidan McNay's page for a little bit of his code. I also copied the way Aidan dispalyed his videos's and images side by side because I liked it.
