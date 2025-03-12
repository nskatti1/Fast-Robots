====================================
Lab 5: Linear PID control and Linear interpolation
====================================

.. contents::
   :depth: 2
   :local:

Attempt 1 
--------------------------------------------------------------------------

I did the lab twice as I messed up the first time and collected the data in python and did the man in python instead of C. I wrote more about my thoughts below in the reflection. Since I did spend a lot of time on this, I still wanted to include it on my page. I can compare and contrast my results.

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

    Kp = 0.06899 # 255 /(4000-304)
    TARGET_DISTANCE = 304  # mm
    DEADBAND = 500 # Stop the car if error is within 100 mm
    MAX_RANGE = 10000  # Maximum detection range
    MAX_PWM = 255  # Full speed
    MIN_PWM = 39  # Ensure movement


Then I create a loop that ran through the commands
I first started with a P Controller, though it did not work the best. My loop in python stored and printed the different values of time, distance, and PWM value.
In my PID loop, I just sent a command that requested the TOF data, calculated the PWM values, and then sent that to the Redboard Nano. Because of the nature of the loop, I sampled my TOF data once every single loop regardless of whether the data was ready or not. I recorded the times at which I sampled these values. That was the frequency at which my PID loop ran. I attached some videos of tbe P controller that I made below. I was not happy with my work and decided to redo the lab.

Prelab
--------------------------------------------------------------------------

Lab
--------------------------------------------------------------------------
Frequency of TOF Sensor and PID Loop 
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


Estimating Distance
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
My mathematical logic was 

:math:`error = distance - target-distance`

:math:`PWM = max(39, min(KP *error, 255))`


Positional Control Task Videos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The following are videos of my PID loop.

Graphs and Extrapolation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The following are videos of my PID loop.

Reflection
-----------------------------
I redid the lab, so I could put the control loop as close to the hardware as possible and try to make a better loop. I found that doing it in python takes time because I had to send the values. While C compiles longer, it executes faster than python. I was very unhappy with my first rersults


Acknowledgements
----------------------------

Credits to Mikayla Lahr for being an awesome TA . I also looked at her solutions and implemented a similar Kp Value(I ended up dividing hers by 5). This got to me a starting place when I had spent a lot of time running my car into a wall. Thanks to Aravind Ramaswami for picking up a battery for me when I battery was bad and Annabel for working along side me when I was figuring out whether I broke my TOF sensor or not(I did not - it was the wrong port). At one point, I asked ChatGPT why my code was not working and it said to check if my battery was unplugged (it was). I used the assistance of ChatGPT for my second attempt as I was in a hurry and decided to redo it 4 hours before it was due after spending the last week trying to get my controller to produce high quality results.
