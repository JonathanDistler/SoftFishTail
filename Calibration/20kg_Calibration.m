clc

%simple script to calibrate the HX711 20 kg load cell
masses=[.115, .340];%masses in kg
calvals=[-205000,-230000]; %calibration values from the calibration script

coefficients = polyfit(masses, calvals, 1);
slope=coefficients(2) %finds the slope

meancoef=mean(calvals)

%mean is -217500

%slope is -192220
