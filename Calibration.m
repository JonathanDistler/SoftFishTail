clf()

%uses script from https://learn.sparkfun.com/tutorials/load-cell-amplifier-hx711-breakout-hookup-guide to calibrate a necessary constant for the 
%hx711 force tester
%masses in kg
mass_1=.211;
mass_2=.311;
mass_3=.411;

%ranges for calibration constants for the HX711 force sensor
mass_1_range=[-325050,-1962050];
mass_2_range=[-2872050,-2052050];
mass_3_range=[-2712050,-2122050];

%creates basis for the the x and y axes
masses=[mass_1,mass_2,mass_3];
range_1=[mass_1_range(1),mass_2_range(1),mass_3_range(1)];
range_2=[mass_1_range(2),mass_2_range(2),mass_3_range(2)];

%mass-averaged difference between the upper and lower range
mass_1_diff=abs(mass_1_range(1)-mass_1_range(2));
mass_1_avg=mass_1_diff/mass_1;


mass_2_diff=abs(mass_2_range(1)-mass_2_range(2));
mass_2_avg=mass_2_diff/mass_2;

mass_3_diff=abs(mass_3_range(1)-mass_3_range(2));
mass_3_avg=mass_3_diff/mass_3;

%compiles averages into an array
mass_avgs=[mass_1_avg,mass_2_avg,mass_3_avg];

%uses polyfit to fit the lines in all instances
slope_upper_bound = polyfit(masses,range_1,1);
upper_slope=slope_upper_bound(1)
slope_lower_bound = polyfit(masses,range_2,1);
lower_slope=slope_lower_bound(1)
slope_avg = polyfit(masses,mass_avgs,1) ;
avg_slop=slope_avg(1)

%just typical average value of the upper and lower bounded slopes for
%calibration
total_avg=(upper_slope+lower_slope)/2

% %%%
% hold on 
% plot(masses,range_1, "LineWidth",2, "DisplayName","Mass vs. Upper-Bound")
% plot(masses,range_2,"LineWidth",2,"DisplayName","Mass vs. Lower-Bound")
% hold off
% 
% xlabel("Mass (kg)")
% ylabel("Calibration-Value (unitless)")
% title("Mass vs. Calibration-Value")
% 
% plot(masses,mass_avgs,"LineWidth",2,"DisplayName","Mass vs. Mass-Averaged Difference")
% % %%%