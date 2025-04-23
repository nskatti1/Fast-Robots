Lab 10: Grid Localization Using Bayes Filter
============================================

.. contents::
   :depth: 2
   :local:

Objective
---------

// The objective of this lab was to implement **grid localization** using the **Bayes Filter**. This method allows a robot to estimate its pose in a discretized space by combining noisy control inputs and sensor observations.

// Localization determines where the robot is within its environment. In each iteration of the Bayes filter, two key steps are performed:

// 1. **Prediction Step** – Incorporates odometry data and actuator noise to predict the new robot location.
// 2. **Update Step** – Uses range measurements (from the 360° rotation) to refine the prediction and reduce uncertainty.

// After each update, the belief over the robot's location is refined, and the cell with the highest belief is interpreted as the most likely pose.

Grid Setup
----------

The robot's world is discretized into a 3D grid defined as:

- **x** ∈ [-1.6764, +1.9812) meters or [-5.5, 6.5) feet  
- **y** ∈ [-1.3716, +1.3716) meters or [-4.5, 4.5) feet  
- **θ** ∈ [-180°, 180°)  

Grid resolution:

- Δx = 0.3048 m  
- Δy = 0.3048 m  
- Δθ = 20°  

This results in a grid of size **(12 × 9 × 18) = 1,944 cells**. The robot is initialized with a point-mass belief at the cell corresponding to (x=0, y=0, θ=0).

Lab Tasks
---------

Implementation
--------------

### Compute Control

// The `compute_control` function calculates the robot’s relative motion from one pose to the next. It returns:

// - `rotation1` (initial rotation)
// - `translation` (straight-line distance)
// - `rotation2` (final rotation)

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

The odometry motion model computes the probability that the robot transitioned from `prev_pose` to `cur_pose`, given the control input `u`. It uses three independent Gaussian distributions to model noise in each motion component.

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

The prediction step loops over all prior grid cells with significant belief (> 0.0001) and spreads that belief across reachable cells using the odometry motion model. Beliefs are normalized afterward to prevent underflow.

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

Simulation Results
------------------

// Each run of the simulation shows:

// - **Green** = Ground truth trajectory
// - **Blue** = Estimated trajectory from belief
// - **Red** = Odometry-only trajectory

// The brighter the grid cell, the higher the belief. After several iterations, the estimated belief converges closely to the ground truth.

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

Video Demo
----------

.. raw:: html

   <iframe width="560" height="315"
           src="https://www.youtube.com/embed/YOUR_VIDEO_ID"
           title="Grid Localization Demo"
           frameborder="0"
           allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
           allowfullscreen>
   </iframe>

Conclusion
----------

// The Bayes Filter successfully localized the robot by combining a probabilistic motion model with a Gaussian sensor model. Performance was accurate in structured environments and degraded slightly with ambiguous sensor readings or symmetric features. Belief maps and trajectories confirmed the method's validity, and the filter’s performance improved as more measurements were incorporated.

Lab 10 References
-----------------

Thanks to the Fast Robots TAs, especially Mikayla Lahr whose webpage I took heavily inspiration from and constantly cross checked.
