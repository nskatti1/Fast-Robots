====================================
Lab 5: Linear PID control and Linear interpolation
====================================

.. contents::
   :depth: 2
   :local:

Attempt 1 
--------------------------------------------------------------------------

I did the lab twice as I messed up the first time and collected the data in python and created the P Controller in python instead of C. I wrote more about my thoughts below in the reflection. Since I did spend a lot of time on this, I still wanted to include it on my page. While it did work, it was not a good controller. My car moved too slowly. My PWM values were between 42 and 39 where 39 is nearly the closest value to stopping.

In C I wrote 2 functions, one function that measured and sent the data back and the other function that took in a PWM value and wrote it to the motors.

.. code-block:: cpp

    case PID_MEASURE: {  

      distanceSensor.startRanging(); 
      while (!distanceSensor.checkForDataReady()) {
        delay(1);
      }
      int curr_distance = distanceSensor.getDistance(); 
      distanceSensor.clearInterrupt();
      distanceSensor.stopRanging();


      tx_characteristic_string.writeValue("");  //  Force reset BLE buffer


      char curr_d[40];  
      String(curr_distance).toCharArray(curr_d, sizeof(curr_d));


      tx_estring_value.clear();  
      tx_estring_value.append(curr_d);
      tx_characteristic_string.writeValue(tx_estring_value.c_str());

      break;
    }



.. code-block:: cpp

    case PID_UPDATE: {  
        int received_pwm = 0;
        success = robot_cmd.get_next_value(received_pwm);
        if (!success) return;

        // Apply received PWM to motors
        if (received_pwm == 0) {
            stop_motors();
        } else {
            move_motors(received_pwm);
        }
        break;
    }

On the flip side, I create a loop in python and set some constants.


.. code-block:: python

    Kp = 0.06899/7 
    TARGET_DISTANCE = 304  
    DEADBAND = 120
    MAX_RANGE = 10000  
    MAX_PWM = 255  
    MIN_PWM = 39  


Then I create a loop that ran through the commands
I first started with a P Controller, though it did not work the best. My loop in python stored and printed the different values of time, distance, and PWM value.
In my PID loop, I just sent a command that requested the TOF data, calculated the PWM values, and then sent that to the Redboard Nano. Because of the nature of the loop, I sampled my TOF data once every single loop regardless of whether the data was ready or not. I recorded the times at which I sampled these values. That was the frequency at which my PID loop ran. I attached some videos of tbe P controller that I made below. I was not happy with my work and decided to redo the lab.

Here are videos of my controller.

Prelab
--------------------------------------------------------------------------
Now for my actual prelab, I used a function to send all my data to python the pid loop runs.

.. code-block:: cpp

void send_pid_data_to_python() {
    tx_characteristic_string.writeValue("");  // Clear buffer

    for (int i = 0; i < sample_count; i++) {
        String data = String(timestamps[i]) + "|" +
                      String(distances[i]) + "|" +
                      String(step_function_distances[i]) + "|" +
                      String(extrapolated_distances[i]) + "|" +
                      String(kp_terms[i], 4) + "|" +
                      String(ki_terms[i], 4) + "|" +
                      String(kd_terms[i], 4) + "|" +
                      String(pwm_values[i]);
        tx_characteristic_string.writeValue(data.c_str());
        delay(50);

I then used my notification handler and the ble.start_notify() function to get my results on python for plotting.
Lab
--------------------------------------------------------------------------
Frequency of TOF Sensor and PID Loop 
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


Estimating Distance
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


Positional Control Task Videos and Corresponding Graphs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The following are videos of my PID loop.

Extrapolation Code
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The following code was used for extrapolation

Reflection
-----------------------------
I redid the lab, so I could put the control loop as close to the hardware as possible and try to make a better loop. I found that doing it in python takes time because I had to send the values. While C compiles longer, it executes faster than python. I was very unhappy with my first results. Overall, I am proud of the work I did. Even though the first part was not a great controller, I am happy I took the time to try to tune it as best I could. I learned a lot and I worked hard. Success!


Acknowledgements
----------------------------

Credits to Mikayla Lahr for being an awesome TA . I also looked at her solutions and implemented a similar Kp Value(I ended up dividing hers by 5). This got to me a starting place when I had spent a lot of time running my car into a wall. Thanks to Aravind Ramaswami for picking up a battery for me when I battery was bad and Annabel for working along side me when I was figuring out whether I broke my TOF sensor or not(I did not - it was the wrong port). At one point, I asked ChatGPT why my code was not working and it said to check if my battery was unplugged (it was). Thanks to Anunth Ramaswami for trying to cheer me on while accross the country as I redid the entire thing.
