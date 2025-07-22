%goal is to parse over the force vs time graphs and determine the ranges for which the motor has stopped working and the range after the motor has stopped working. conditions are semi-arbitrary. A lot of them were hardcoded visually. Next step would be to implement the cv script to determine when it touches a wall-fish's head angle too great
figure;  % creates a new figure each loop
for index=2:.2:3.8
    roundedIndex = round(index, 1);
    
    filename = sprintf("C:\\Users\\15405\\OneDrive\\Desktop\\Career\\ETHZ\\ETHZ Work\\HardwareOutput\\%.1f_unfiltered.csv", roundedIndex);
    %filename = sprintf("C:\\Users\\15405\\OneDrive\\Desktop\\Career\\ETHZ\\ETHZ Work\\HardwareOutput\\%.1f_unfiltered.csv", roundedIndex);
    data = readtable(filename);

    rawForce = string(data{3:end, 1});
    % Clean: remove 'kg' and whitespace
    cleanForce = str2double(erase(lower(strtrim(rawForce)), "kg"));
    %Cleans up time data 
    rawTime = string(data{3:end, 2});
    cleanTime=str2double(rawTime);

    %semi-arbitrary conditions to filter out starting forces and the return
    %state 
    index_tolerance=50;
    value_tolerance=.075;

    max_time_tolerance=27.5;
    min_time_tolerance=5;


    [pks,locs]=findpeaks(cleanForce);
    min_vals=[];
    min_vals_2=[];

    for index=1:length(pks)
        if locs(index)>index_tolerance & abs(pks(index))<value_tolerance
            time_index=locs(index);
            if cleanTime(time_index)<max_time_tolerance
                min_vals=[min_vals,locs(index)];
            end
        end

        if abs(pks(index))<value_tolerance
            time_index=locs(index);
            if cleanTime(time_index)>min_time_tolerance
                min_vals_2=[min_vals_2,(time_index)];
            end
        end

    end

    max_index=min_vals(1)
    min_index=min_vals_2(1)

    
    % Check if max_index is too close to min_index or same as it
    if max_index == min_index || (max_index - min_index) < 10
        max_index = min_vals_2(2);
    end


    time_vals=cleanTime(min_index:max_index);
    force_vals=cleanForce(min_index:max_index);

    %plots force vs time for each respective hertz
    plot(time_vals,force_vals)
    title(sprintf('%.1f Hz: Time vs Force', roundedIndex));
    xlabel('Time (s)');
    ylabel('Force (kg)');

    % Save figure to its respective folder
    folder = 'C:\Users\15405\OneDrive\Desktop\Career\ETHZ\ETHZ Work\HardwareOutput';
    filenameFig = fullfile(folder, sprintf('%.1f_HZ_TimevsForce_Cleaned.fig', roundedIndex));
    saveas(gcf, filenameFig);



end



