%Same as Calibration.m, however, this is for the updated 1kg loadcell from HK711 force tester
clf()

%uses script from https://learn.sparkfun.com/tutorials/load-cell-amplifier-hx711-breakout-hookup-guide to calibrate a necessary constant for the 
%hx711 force tester

%linear interpolation formula
%y1+(y2-y1)/(x2-x1)*(x-x1)

%masses in kg, the first number represents the measured mass
%the other masses provide range for linear interpolation
mass_1=.311;
x2_1=.4;
x1_1=.3;

%ranges for calibration constants for the HX711 force sensor
mass_1_range=[-3347500,-2407500];

%gives a linear interpolation to find the interpolated value for the
%calibration value of the first mass 
cal_1=mass_1_range(1)+((mass_1_range(2)-mass_1_range(1))/(x2_1-x1_1))(*mass_1-x1_1)

mass_2=.411;
x2_2=.5;
x1_2=.4;
mass_2_range=[-3177500,-2477500];

cal_2=mass_2_range(1)+((mass_2_range(2)-mass_2_range(1))/(x2_2-x1_2))*(mass_2-x1_2)

mass_3=.261;
x2_3=.3;
x1_3=.2;
mass_3_range=[-6487500,-3897500];

cal_3=mass_3_range(1)+((mass_3_range(2)-mass_3_range(1))/(x2_3-x1_3))*(mass_3-x1_3)



%creates basis for the the x and y axes
masses=[mass_1,mass_2,mass_3];

%calibrated array
cals=[cal_1,cal_2,cal_3];

%uses polyfit to fit the lines in all instances (1st degree polynomial)
slope_fit = polyfit(masses,cals,1);
slope_val=slope_fit(1)


hold on 
plot(masses,cals, "LineWidth",2, "DisplayName","Mass vs. Calibration-Value")
hold off

xlabel("Mass (kg)")
ylabel("Calibration-Value (unitless)")
title("Mass vs. Calibration-Value")

