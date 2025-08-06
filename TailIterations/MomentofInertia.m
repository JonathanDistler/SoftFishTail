clc;
clf;

rod=1;
slab=2;
clear length;

%code takes a bunch of evenly spaced points and derives the moment of inertia of a  specific geometry based on a certain geometry of choice (rod or slab). If b is the same value as L, in the limit as a approaches 0, the two results will be the same


M_arr=linspace(.01,.04,25);
L_arr=linspace(.01,.025,25);
a_arr=linspace(.004,.01,25);
b_arr = linspace(.025, .04, 25);
d_arr = linspace(.0125, .0110, 25);


%%%%Write the number down for the moment of inertia body you want to
%%%%evaluate
user_input=2; %in this case, it would be a thin_rod



if user_input == 1
   disp("Thin-Rod")
   mass_arr=[];
   length_arr=[];
   distance_arr=[];
   moment_of_inertia_arr=[];
   for i = 1:length(M_arr)
       mass = M_arr(i);
       for j = 1:length(L_arr)
           length_val = L_arr(j);
           for k = 1:length(d_arr)
               %defines the 3 independent variables at each time step and
               %assigns to an arry
               %calculates the MoI with teh function for slab
               distance = d_arr(k);
               moment_of_inertia=thinRodMoI(mass, length_val, distance);
               moment_of_inertia_arr(end+1)=moment_of_inertia;
               mass_arr(end+1)=mass;
               distance_arr(end+1)=distance;
               length_arr(end+1)=length_val;
           end
       end
   end

   figure;
   %x, y, z, marker size
   scatter3(distance_arr, mass_arr, moment_of_inertia_arr, length_arr*700,'filled');
   % Axis labels
   xlabel('Distance');
   ylabel('Mass');
   zlabel('Moment of Inertia');
   title('3D Scatter: Distance vs Mass vs Moment of Inertia');
   colorbar;
   colormap jet;
   view(45, 25); % adjust view angle for better 3D perspective
   grid on;


elseif user_input==2
   disp("Thin-Slab")
   moment_of_inertia_arr=[];
   distance_arr=[];
   mass_arr=[];
   a_arr2=[];
   b_arr2=[];
 
   for i=1:length(M_arr)
       mass=M_arr(i)
       for j=1:length(a_arr)
           a=a_arr(j)
           for k=1:length(b_arr)
               b=b_arr(k)
               for l=1:length(d_arr)
                   %defines the 4 independent variables at each time step
                   %and assigns to an array
                   %calculates the MoI with the function for slab
                   dist=d_arr(l);
                   moment_of_inertia=thinSlabMoI(mass, a, b, dist);
                   moment_of_inertia_arr(end+1)=moment_of_inertia;
                   distance_arr(end+1)=dist;
                   mass_arr(end+1) = mass;
                   a_arr2(end+1) = a;
                   b_arr2(end+1) = b;
    
               end
           end
       end
   end
   figure;
   %x, y, z, color, marker size
   scatter3(distance_arr, mass_arr, moment_of_inertia_arr, b_arr2*200, a_arr2*700, 'filled');
   % Axis labels
   xlabel('Distance');
   ylabel('Mass');
   zlabel('Moment of Inertia');
   title('4D Scatter: Distance vs Mass vs Moment of Inertia');
   colorbar;
   colormap jet;
   view(45, 25); % adjust view angle for better 3D perspective
   grid on;
       
else
   disp("Not a valid moment of inertia body");
end
function [moi]=thinRodMoI(mass, length_val, distance)
   base_moi = (1/12) * mass * length_val^2;
   moi = base_moi + mass * distance^2; % Apply parallel axis theorem
end
function [moi]=thinSlabMoI(mass, a_dist, b_dist, distance)
   base_moi = (1/12) * mass * (a_dist^2 + b_dist^2);
   moi = base_moi + mass * distance^2;
end

