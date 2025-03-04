====================================
Lab 4: Motors and Open Loop Control
====================================

.. contents::
   :depth: 2
   :local:

Prelab
--------------------------------------------------------------------------
Here is a diagram of my wiring for lab 4. I used analog pins A2 and A3 for one motor driver and analog pins A4 and A5 for the motor driver on the Artemis board. The ground pins below A2 and A4 grounded the motor drivers.

.. image:: images/l4_diagram.png
   :align: center
   :width: 50%
   :alt: Wiring Diagram 



I powered the motordriver/motors and the Artemis from separate batteries because one battery does not provide enough current for both the Artemis and the motors/motor drivers.

Lab
--------------------------------------------------------------------------

Motor Driver Setup and Testing
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

First thing I did was solder my motor controllers. I soldered them according to the wiring diagram above. The following pictures are all photos of what I did 



.. list-table::
   :widths: auto
   :align: center

   * - .. image:: images/l4_pic_1.jpg
          :width: 200px
     - .. image:: images/l4_pic_2.jpg
          :width: 200px
     - .. image:: images/l4_pic_3.jpg
          :width: 200px


.. image:: images/l4_pic_5.jpg
   :align: center
   :width: 50%
   :alt: Pics

.. image:: images/l4_pic_6.jpg
   :align: center
   :width: 50%
   :alt: Pics

.. image:: images/l4_pic_7.jpg
   :align: center
   :width: 50%
   :alt: Pics


.. list-table::
   :widths: auto
   :align: center

   * - .. image:: images/l4_pic_1.jpg
          :width: 50%
          :alt: Pic 1
     - .. image:: images/l4_pic_2.jpg
          :width: 50%
          :alt: Pic 2
     - .. image:: images/l4_pic_3.jpg
          :width: 50%
          :alt: Pic 3
   * - .. image:: images/l4_pic_5.jpg
          :width: 50%
          :alt: Pic 5
     - .. image:: images/l4_pic_6.jpg
          :width: 50%
          :alt: Pic 6
     - .. image:: images/l4_pic_7.jpg
          :width: 50%
          :alt: Pic 7

.. grid:: 3
   :gutter: 1

   .. grid-item::
      .. image:: images/l4_pic_1.jpg
         :width: 100%
         :alt: Pic 1

   .. grid-item::
      .. image:: images/l4_pic_2.jpg
         :width: 100%
         :alt: Pic 2

   .. grid-item::
      .. image:: images/l4_pic_3.jpg
         :width: 100%
         :alt: Pic 3

.. grid:: 3
   :gutter: 1

   .. grid-item::
      .. image:: images/l4_pic_5.jpg
         :width: 100%
         :alt: Pic 5

   .. grid-item::
      .. image:: images/l4_pic_6.jpg
         :width: 100%
         :alt: Pic 6

   .. grid-item::
      .. image:: images/l4_pic_7.jpg
         :width: 100%
         :alt: Pic 7


tezr
.. raw:: html

   <div style="display: flex; justify-content: center; gap: 10px;">
       <img src="images/l4_pic_5.jpg" width="30%" alt="Image 1">
       <img src="images/l4_pic_6.jpg" width="30%" style="transform: rotate(90deg);" alt="Rotated Image 2">
       <img src="images/l4_pic_7.jpg" width="30%" alt="Image 3">
   </div>


.. image:: images/l4_oscop1.jpg
   :align: center
   :width: 50%
   :alt: Oscope 1

.. image:: images/l4_oscop2.jpg
   :align: center
   :width: 50%
   :alt: Oscop 2


.. image:: images/l4_oscop3.jpg
   :align: center
   :width: 50%
   :alt: Oscop 3

Taking the Car Apart and Powering the Motors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
I took the car apart. Here is a photo of me saving the screws. I used an old open pouch for it.

l4_screw.jpg
.. image:: images/l4_screw.jpg
   :align: center
   :width: 50%
   :alt: Screws Storage


Here is a video of me testing my motor controller/motor functionality before installing the car.

.. youtube:: https://youtube.com/shorts/FDdRFmgxxyc
   :width: 560
   :height: 315

Afterwards, I repeated this with the battery.


Installing the Car
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Testing the Car
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^



Reflection
-----------------------------





Acknowledgements
-----------------------------
Thank you to Anunth Ramaswami for sitting with me while I soldered and for bringing me a power supply for when I tested powering my motors and letting me use your lab to save walking time. Thank you to Aravind Ramaswami for bringing the power supply for when I was using the oscillosope and letting me use your lab to save walking time. Thank you to Sabian Grier, Becky Lee, Aidan McNay, Paige Shelton, and Annabel Lian for hyping me up and making sure I ate enough food to fuel me through working on this lab. I used ChatGPT to help figure grid images and edit my conf.py file in regards to my website. Turns out videos were not appearing because I did not edit my extensions.
