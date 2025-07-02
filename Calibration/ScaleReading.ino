/*
 Example using the SparkFun HX711 breakout board with a scale
 By: Nathan Seidle
 SparkFun Electronics
 Date: November 19th, 2014
 License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).
 
 This example demonstrates basic scale output. See the calibration sketch to get the calibration_factor for your
 specific load cell setup.
 
 This example code uses bogde's excellent library: https://github.com/bogde/HX711
 bogde's library is released under a GNU GENERAL PUBLIC LICENSE
 
 The HX711 does one thing well: read load cells. The breakout board is compatible with any wheat-stone bridge
 based load cell which should allow a user to measure everything from a few grams to tens of tons.

 Arduino pin 2 -> HX711 CLK
 3 -> DAT
 5V -> VCC
 GND -> GND
 
 The HX711 board can be powered from 2.7V to 5V so the Arduino 5V power should be fine.
 
*/


#define calibration_factor -2694200 //This value is obtained using the SparkFun_HX711_Calibration sketch

#include "HX711.h"

#define DOUT  2
#define CLK   3

HX711 scale;  // Use default constructor

void setup() {
  Serial.begin(9600);
  Serial.println("HX711 scale demo");
  scale.begin(DOUT, CLK);  // Initialize pins here
  scale.set_scale(calibration_factor); //This value is obtained by using the SparkFun_HX711_Calibration sketch
  scale.tare();	//Assuming there is no weight on the scale at start up, reset the scale to 0
  
  Serial.println("Readings:");
}

void loop() {
  Serial.print("Reading: ");
  Serial.print(scale.get_units(), 4); //scale.get_units() returns a float
  Serial.println(" KG"); //You can change this to kg but you'll need to refactor the calibration_factor
  Serial.print(scale.get_units()*9.8,4);
  Serial.println(" N");
  Serial.println();
}
