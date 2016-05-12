/*
  Receives messages from the segbot and alters the LED lights in the desired manner. 
  Possible messages: 
    '0' - default, all lights on the LED are active
    '1' - left turn signal, lights on the left half of the robot blink
    '2' - right turn signal, lights on right half of the robot blink 
   
   To use this sketch, it must be uploaded to the Arduino board and the following command must be run on the command line to open the serial port for communication
   between the sketch and the topic
     rosrun rosserial_python serial_node.py /dev/ttyACM1 
   
   Code adapted from LedStripColorTester.ino by David Grayson
   Code can be found at https://github.com/pololu/pololu-led-strip-arduino/blob/master/PololuLedStrip/examples/LedStripColorTester/LedStripColorTester.ino
*/
#include <ros.h>
#include <std_msgs/Char.h>
#include <PololuLedStrip.h>

// Create an ledStrip object and specify the pin it will use.
PololuLedStrip<12> ledStrip; // use pin 12 

// Create a buffer for holding the colors (3 bytes per color).
#define LED_COUNT 60
rgb_color colors[LED_COUNT];

#define DEFAULT '0'
#define LEFT '1'
#define RIGHT '2'
#define LEFT_START 29
#define LEFT_END 59
#define RIGHT_START 0
#define RIGHT_END 28
#define TIME_DELAY 500 //Half a second 

ros::NodeHandle n; 
char setting = '0'; 

// Receives the current setting from the robot. 
void signal_cb(const std_msgs::Char& msg){
  setting = msg.data; 
}

// Setup subscriber 
ros::Subscriber<std_msgs::Char> sub("/led_strip/turn_signal", &signal_cb);

// Fills an array with "burnt orange" color objects. Creates the default setting (all lights on and set to burnt orange color). 
void setColor() {
  rgb_color color;
  color.red = 255;
  color.green = 69;
  color.blue = 0;
  
  // Updates the colors buffer 
  for(int i = 0; i < LED_COUNT; i++) {
    colors[i] = color; 
  }
}

// Sets the specified portion of the color array to have no color (lights turned off). 
void createTurnSignal(int start, int end) {
  rgb_color oldColor = colors[start]; 
  rgb_color off;
  off.red = 0;
  off.green = 0;
  off.blue = 0;

  // Update the colors buffer.
  for(int i = start; i < end; i++)
  {
     colors[i] = off;
  }
}

void setup()
{
  // Sets up subscriber and sets all lights on the strip to "burnt orange" 
  n.initNode(); 
  n.subscribe(sub); 
  setColor(); 
}

void loop()
{
  n.spinOnce(); 
  int start; 
  int end; 
  // Determine portion of lights that need to be turned off 
  if(setting == LEFT) {
    start = LEFT_START;
    end = LEFT_END;
  }
  else if(setting == RIGHT) {
    start = RIGHT_START;
    end = RIGHT_END;
  }
  else {
    start = 0;
    end = 0; 
  }
  
  // Copies "default" setting 
  rgb_color originalColors[LED_COUNT]; 
  memcpy(&originalColors, &colors, sizeof colors);
  
  // Turns a portion of the lights off 
  createTurnSignal(start, end);
  ledStrip.write(colors, LED_COUNT);
  
  //Restore original colors after a short delay 
  memcpy(&colors, &originalColors, sizeof colors);
  delay(TIME_DELAY); 
  ledStrip.write(colors, LED_COUNT); 
}


