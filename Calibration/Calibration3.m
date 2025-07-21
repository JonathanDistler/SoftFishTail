%final update on the HX711 calibration
%goes away from linear interpolation method to a more accurate 3 significant-figure system
%no longer having to worry about significant figure issues. Can just roughly average out the values to get a pretty good calibration value

%Same as Calibration.m, however, this is for the updated 1kg loadcell from HK711 force tester
%For the 3kg loadcell, it produces a calibrated value of -887980 
clf()

%uses script from https://learn.sparkfun.com/tutorials/load-cell-amplifier-hx711-breakout-hookup-guide to calibrate a necessary constant for the 
%hx711 force tester

%linear interpolation formula
%y1+(y2-y1)/(x2-x1)*(x-x1)

%masses in kg, the first number represents the measured mass
%the other masses provide range for linear interpolation
mass_1=.361;
x2_1=.362;
x1_1=.360;

%ranges for calibration constants for the HX711 force sensor
mass_1_range=[-2697100,-2687100];

cal_1=(mass_1_range(1)+mass_1_range(2))/(2)

mass_2=.411;
cal_2=-2697100
%accuracy took it exactly to .411 kg

mass_3=.311;
x2_3=.313;
x1_3=.310;
mass_3_range=[-2697100,-2677100];

cal_3 = mass_3_range(1) + ((mass_3_range(2) - mass_3_range(1)) / (x2_3 - x1_3)) * (mass_3 - x1_3)


mass_4=.261;
cal_4=-2697100

%creates basis for the the x and y axes
masses=[mass_4,mass_3,mass_1, mass_2];

%calibrated array
cals=[cal_4,cal_3,cal_1,cal_2];

%averages all of the values
sum_val = 0;
for i = 1:4
    sum_val = sum_val + cals(i);
end
calibration_val=sum_val/length(cals)
%calibration value=-2694200

