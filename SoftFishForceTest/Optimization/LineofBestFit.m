%goal of this script is to take the stiffness and damping values from the Nelder Mead optimization and produce lines of best fit

%tendon stiffness had been -1.66e-6 ~0
ctrlrange=[-4,-3.5,-3,-2.5,-2,-1.5,-1,-.5]
tendonstiff=[0,0,0,0,0, 0,0, 0]
tendondamp=[2.00616060e1,2.062569e1,2.04391875e1,1.95948124e1, 2.05185604e1,2.05185604e1, 1.983333566e1, 2.01313248e1]
jointstiff=[6.04601253e1,6.04744814e1,6.13475361e1,6.35579502e1,6.1674853e1,6.1674853e1, 6.16165395e1, 5.97940750e1]
jointdamp=[3.12228865e1,2.95608593e1,2.92225726e1,3.04812405e1,3.07824431e1,3.07824431e1, 3.032784e1, 2.98012505e1]


ts2= polyfit(ctrlrange, tendonstiff, 2) %should always be zero -duh

%2nd and 3rd order polynomial line of best fit for tension dampness
td2= polyfit(ctrlrange, tendondamp, 2)
td3= polyfit(ctrlrange, tendondamp, 7)

%2nd and 3rd order polynomial line of best fit for joint stiffness
js2=polyfit(ctrlrange, jointstiff, 2)
js3=polyfit(ctrlrange, jointstiff, 7)

%2nd and 3rd order polynomial line of best fit for joint dampness
jd2=polyfit(ctrlrange, jointdamp, 2)
jd3=polyfit(ctrlrange, jointdamp, 7)

x_fit = linspace(min(ctrlrange), max(ctrlrange), 100); % Generate points for smooth curve
td2fit = polyval(td2, x_fit);
td3fit = polyval(td3, x_fit);

js2fit = polyval(js2, x_fit);
js3fit = polyval(js3, x_fit);

jd2fit = polyval(jd2, x_fit);
jd3fit = polyval(jd3, x_fit);

%would plot tendon stiffness, but there's no point, it is completely
%horizontal 

%plotting 2nd order polynomial and 7th order because 7th order is the first
%time that the line of best fit-crosses through all of the different
%data-points 

% Plot the data and the fit
plot(ctrlrange, tendondamp, 'o', x_fit, td2fit, '-',x_fit,td3fit,"*");
legend('Data Points', 'Polynomial-2 Fit', 'Polynomial-7 Fit');
title('Polynomial Fit Example Tendon Damping');
xlabel('Control Range');
ylabel('y');

% Plot the data and the fit
plot(ctrlrange, jointstiff, 'o', x_fit, js2fit, '-',x_fit,js3fit,"*");
legend('Data Points', 'Polynomial-2 Fit', 'Polynomial-7 Fit');
title('Polynomial Fit Example Joint Stiffness');
xlabel('Control Range');
ylabel('Y');

% Plot the data and the fit
plot(ctrlrange, jointdamp, 'o', x_fit, jd2fit, '-',x_fit,jd3fit,"*");
legend('Data Points', 'Polynomial-2 Fit', 'Polynomial-7 Fit');
title('Polynomial Fit Example Joint Damping');
xlabel('Control Range');
ylabel('y');
