%goal of this code is to sum over all of the maximum forces and average them out to see what frequency produces the most x and y component forces

freq=[]
real_freq=[]
force_x=[]
force_y=[]
force_tot=[]
clear sum
for index=2.2:.2:3.8
    sum_time=[];
    figure;  % creates a new figure each loop
    loop_max=0;
    % Round index to 1 decimal place to avoid floating point issues
    roundedIndex = round(index, 1);
    filename = sprintf("C:\\Users\\15405\\HardwareOutput\\CV_Tail_Track\\InterpolatedHeadForceData_%.1f.csv", roundedIndex);
    data = readtable(filename);

    freq=[freq,roundedIndex];
    
    % Extract columns as numeric arrays using curly braces {}, not parentheses ()
    data_t=data{:,1};
    data_x = data{:, 2};    % 2nd column (force x)
    data_y = data{:, 3};    % 3rd column (force y)
    data_tot = data{:, 4};  % 4th column (total force)
    
    % Now find peaks
    [pks_x, locs_x] = findpeaks(data_x);
    [pks_y, locs_y] = findpeaks(data_y);
    [pks_tot, locs_tot] = findpeaks(data_tot);

    for index_val=1:length(pks_tot) -2
        time_index=locs_tot(index_val);
        time_val=data_t(time_index);
        time_val_2=data_t(time_index+2);
        delta_time=time_val_2-time_val;
        sum_time=[sum_time,delta_time];
    end

    avg_freq=mean(sum_time);

    mean_x=mean(pks_x);
    mean_y=mean(pks_y);
    mean_tot=mean(pks_tot);

    force_x=[force_x,mean_x];
    force_y=[force_y,mean_y];
    force_tot=[force_tot,mean_tot];

end

bar(freq, force_x, 'DisplayName', 'x-force-component'); 
title('Frequency vs Average Maximum Force -X');
xlabel('Frequency (hZ)');
ylabel('Force (kg)');
% Save figure to its respective folder
folder = 'C:\Users\15405\OneDrive\Desktop\Career\ETHZ\ETHZ Work';
filenameFig_1 = fullfile(folder, 'FreqvsForce_x.fig');
saveas(gcf, filenameFig_1);
clf;
bar(freq, force_y, 'DisplayName', 'y-force-component');
title('Frequency vs Average Maximum Force -Y');
xlabel('Frequency (hZ)');
ylabel('Force (kg)');
filenameFig_2 = fullfile(folder, 'FreqvsForce_y.fig');
saveas(gcf, filenameFig_2);
clf;
bar(freq, force_tot, 'DisplayName', 'total-force-component');
title('Frequency vs Average Maximum Force -Total');
xlabel('Frequency (hZ)');
ylabel('Force (kg)');
filenameFig_3 = fullfile(folder, 'FreqvsForce_tot.fig');
saveas(gcf, filenameFig_3);
clf;




