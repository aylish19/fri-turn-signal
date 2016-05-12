# fri-turn-signal
Final project code for CS378 - Autonomous Intelligent Robotics

Code for implementing turns signals on the UT-Austin Builiding Wide Intelligence (BWI) segway bots. 

To use: 
  1. Install rosserial_arduino using the following commands if it not already installed.  
  
      sudo apt-get install ros-indigo-rosserial-arduino

      sudo apt-get install ros-indigo-rosserial

  2. Upload turn_signal.ino to an Arduino board connected to a Polulu Light Strip (preferably connected to Pin 12).

  3. Add package "fri_led_strip" to workspace and compile 

  4. Run segbot launch file

  5. Kill any nodes that may interfere with serial communication
  
     rosnode kill /arduino_drivers

     rosnode kill /battery_diagnostics

  5. Open serial port by using the following command 
  
      rosrun rosserial_python serial_node.py /dev/ttyACM1

  6. Run the turn monitoring node by using the following command 
  
      rosrun fri_led_strip turn_monitor 
