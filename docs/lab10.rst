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

   // def compute_control(cur_pose, prev_pose):
   //     dx = cur_pose[0] - prev_pose[0]
   //     dy = cur_pose[1] - prev_pose[1]
   //     rotation1 = np.arctan2(dy, dx) - prev_pose[2]
   //     translation = np.hypot(dx, dy)
   //     rotation2 = cur_pose[2] - prev_pose[2] - rotation1
   //     return normalize_angle(rotation1), translation, normalize_angle(rotation2)

### Odometry Motion Model

The odometry motion model computes the probability that the robot transitioned from `prev_pose` to `cur_pose`, given the control input `u`. It uses three independent Gaussian distributions to model noise in each motion component.

.. code-block:: python

   // def odom_motion_model(cur_pose, prev_pose, u):
   //     rot1, trans, rot2 = compute_control(cur_pose, prev_pose)
   //     return gaussian(rot1, u[0], rot_noise) * \
   //            gaussian(trans, u[1], trans_noise) * \
   //            gaussian(rot2, u[2], rot_noise)

### Prediction Step

The prediction step loops over all prior grid cells with significant belief (> 0.0001) and spreads that belief across reachable cells using the odometry motion model. Beliefs are normalized afterward to prevent underflow.

.. code-block:: python

   // if prior_belief > 0.0001:
   //     bel_bar[x_new, y_new, theta_new] += \
   //         bel[x_prev, y_prev, theta_prev] * \
   //         odom_motion_model(...)

### Sensor Model

Each observation consists of 18 distance readings. For each grid cell, the expected readings are compared to the observed readings using a Gaussian likelihood function (per reading), assuming conditional independence.

.. code-block:: python

   // prob_array = gaussian(obs_range_data - expected_ranges, 0, obs_noise)
   // total_likelihood = np.prod(prob_array)

### Update Step

The update step multiplies the predicted belief (`bel_bar`) by the sensor likelihood and normalizes the result.

.. code-block:: python

   // loc.bel = loc.bel_bar * sensor_likelihood
   // loc.bel /= np.sum(loc.bel)

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
