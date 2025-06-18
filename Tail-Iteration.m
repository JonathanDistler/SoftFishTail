%program that iterates over different values for length and width of a
%soft-robotic fish tail. This will help illuminate different tail
%dimensions to minimize or maximize the radii-of curvature

clear max
syms x 

def = 0;
optimal_width = 0;
optimal_length = 0;

deflections = [];

length_bound = 0.3;
width_bound = 0.3;

step_length = 0.1;
step_width = 0.1;

% constants
density = 1052; % kg/m^3
P = 10;         % N
E = 0.069e9;    % Pa (GPa to Pa)

for i = 0.1:step_length:length_bound
    for j = 0.1:step_width:width_bound

        l = i;
        w = j;

        volume = l * w;
        m = density * volume;

        I_com = m * w^2 / 12;
        I_total = I_com + m * (l / 2)^2;

        delta = (P * l^2) / (3 * E * I_total);

        if delta > def
            def = delta;
            optimal_length = l;
            optimal_width = w;
        end

        deflections(end + 1) = delta;
    end
end

disp(deflections)
[Value, Index] = max(deflections);

% sanity check
volume = optimal_length * optimal_width;
m = density * volume;
I_com = m * optimal_width^2 / 12;
I_total = I_com + m * (optimal_length / 2)^2;

delta = (P * optimal_length^2) / (3 * E * I_total);
disp("Sanity check delta = " + num2str(delta))
if (Value==delta)
    disp("True")
end

equation=-P*x^2/(6*E*I_total)*(3*l-x);
d_equation=diff(equation,x);
dd_equation=diff(d_equation,x);
%d_equation=-2*P*x/(6*E*I_total)-P*x^2/(6*E*I_total)*-3*l;
radius_of_curvature=((1+(d_equation)^2)^(3/2))/(dd_equation)
simplify(subs(radius_of_curvature, x, optimal_length))

x_vals = 0.1:step_length:length_bound-step_length;
radius_vals = double(subs(radius_of_curvature, x, x_vals));

figure;
plot(x_vals, radius_vals, 'LineWidth', 2, 'DisplayName', 'Radius of Curvature');
xlabel('x (m)')
ylabel('Radius of Curvature (m)')
title('Radius of Curvature Along Beam')
legend show
grid on


