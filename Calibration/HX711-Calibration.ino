/*
 Example using the SparkFun HX711 breakout board with a scale
 By: Nathan Seidle
 SparkFun Electronics
 Date: November 19th, 2014
 License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

 This is the calibration sketch. Use it to determine the calibration_factor that the main example uses. It also
 outputs the zero_factor useful for projects that have a permanent mass on the scale in between power cycles.

 Setup your scale and start the sketch WITHOUT a weight on the scale
 Once readings are displayed place the weight on the scale
 Press +/- or a/z to adjust the calibration_factor until the output readings match the known weight
 Use this calibration_factor on the example sketch

 This example assumes pounds (lbs). If you prefer kilograms, change the Serial.print(" lbs"); line to kg. The
 calibration factor will be significantly different but it will be linearly related to lbs (1 lbs = 0.453592 kg).

 Your calibration factor may be very positive or very negative. It all depends on the setup of your scale system
 and the direction the sensors deflect from zero state

 This example code uses bogde's excellent library: https://github.com/bogde/HX711
 bogde's library is released under a GNU GENERAL PUBLIC LICENSE

 Arduino pin 2 -> HX711 CLK
 3 -> DOUT
 5V -> VCC
 GND -> GND

 Most any pin on the Arduino Uno will be compatible with DOUT/CLK.

 The HX711 board can be powered from 2.7V to 5V so the Arduino 5V power should be fine.

*/

#include "HX711.h"

#define DOUT  2
#define CLK   3

HX711 scale;  // Use default constructor

float calibration_factor = -300000; // -7050 worked for my 440lb max scale setup

void setup() {
  Serial.begin(9600);
  Serial.println("HX711 calibration sketch");
  Serial.println("Remove all weight from scale");
  Serial.println("After readings begin, place known weight on scale");
  Serial.println("Press + or a to increase calibration factor");
  Serial.println("Press - or z to decrease calibration factor");

  scale.begin(DOUT, CLK);  // Initialize pins here

  scale.set_scale();  // Initialize scale with no calibration factor yet
  scale.tare();       // Reset the scale to 0

  long zero_factor = scale.read_average(); // Get a baseline reading
  Serial.print("Zero factor: ");  // Useful for permanent scale projects
  Serial.println(zero_factor);
}

void loop() {
  scale.set_scale(calibration_factor); // Apply calibration factor

  Serial.print("Reading: ");
  Serial.print(scale.get_units(), 4); //I adjusted the number of significant figure units
  Serial.print(" kg");  // Change to "kg" and adjust calibration if needed
  Serial.print(" calibration_factor: ");
  Serial.print(calibration_factor);
  Serial.println();

  if (Serial.available()) {
    char temp = Serial.read();
    if (temp == '+' || temp == 'a')
      calibration_factor += 1000;
    else if (temp == '-' || temp == 'z')
      calibration_factor -= 1000;
  }
}
