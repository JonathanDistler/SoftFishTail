clc
clf

%function to derive position (alpha) wrt the center of the motor for the first linkage in a soft-fish tail. Uses Matlab function used throughout Dr. Savransky's Cornell MAE Dynamics course
function dz = springfish(t, z, k, h, d, w, l0, M, I)
    %from hand calcs, theta is the angular velocity times time. . . with cw
    %it just moves the graph forward pi/2 radians
    theta = w * t;

    %defining first order state space, with angle being alpha and angular
    %velocity being alphadot
    alpha = z(1);
    alphadot = z(2);
    
    %just defining a lot of the numerators and denominators from my
    %hand-calcs
    denom = sqrt((h + d * sin(theta))^2 + (d - d * cos(theta))^2);
    num = -k * (abs(h + d * sin(theta)) + abs(d - d * cos(theta)) - l0);

    num1 = h + d * sin(theta);
    num2 = d - d * cos(theta);

    mult1 = d * (cos(theta) * cos(alpha) + sin(theta) * sin(alpha));
    mult2 = -d * (cos(theta) * sin(alpha) - sin(theta) * cos(alpha));
    
    %sets up second order state space (angular velocity, angular
    %acceleration)-written in terms of first order state space variables
    dz = [alphadot;
          (mult1 * num * num1 / denom + mult2 * num * num2 / denom) / I];
end

k = 100; %spring constant, can try different iterations in limit towards inf
h = 0.1; %height between the two linkages, rough order of magnitude of real-world
d = 0.2; %width of the first linkage
w = 10; %angular velocity in rad/s. Don't know if this is super accurate, provides good basis
l0 = h; %spring unstretched length, same as distance between the two. . .could try preloading, too (ie greater than or less than distance between)
M = 1; %moment wrt center of motor. . . need anecdotal data to inform this moment
I = 1; %moment of inertia wrt center of motor. . .need to gather this physically, too

hold on
for i=10:5:50
    w=i;
    f = @(t, z) springfish(t, z, k, h, d, w, l0, M, I);
    [t, z] = ode45(f, [0 10], [0; 0]);
    plot(t, z(:,1), 'LineStyle', ':', "LineWidth", 2, "DisplayName", sprintf("Angular Velocity (w%d)",w));
    xlabel("Time (s)");
    ylabel("Angle (rad)");
    titlestr = ("Position vs. Time of First Linkage w.r.t. Motor");
    title(titlestr);
    legend("location","northwest");

end
hold off
