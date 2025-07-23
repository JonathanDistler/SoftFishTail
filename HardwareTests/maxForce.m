%parses through all of my different frequency files to determine the maximum force output-it did require specfici naming conventions of the csvs, mainly {frequency.0}.csv
clear max
max_val=0;
%frequency increases at .1 hertz at a time from 1 to 4 hz
for index=1.0:.2:4.0
    loop_max=0;
    % Round index to 1 decimal place to avoid floating point issues
    roundedIndex = round(index, 1);
    filename = sprintf("C:\\Users\\15405\\OneDrive\\Desktop\\Career\\ETHZ\\ETHZ Work\\HardwareOutput\\Real-to-Sim-tests\\%.1f.csv", roundedIndex);
    data = readtable(filename);

    % Extract column 2 as strings
    rawForce = string(data{:, 2});  % NOT data(:,2) — use {} to get contents

    % Clean: remove 'kg' and whitespace
    cleanForce = str2double(erase(lower(strtrim(rawForce)), "kg"));

    % Get max
    maxVal = max(cleanForce);
    minVal=min(cleanForce);

    if abs(minVal)>maxVal
        loop_max=abs(minVal);

    else
        loop_max=maxVal
    end
    
    disp(loop_max);

    if loop_max>max_val
        disp(loop_max);
        max_val=loop_max;
    end
    

end
disp(max_val);

%now the frequecy increases by .2 hertz at a time
for index=2.2:.2:4.0
    loop_max=0;
    % Round index to 1 decimal place to avoid floating point issues
    roundedIndex = round(index, 1);
    filename = sprintf("C:\\Users\\15405\\OneDrive\\Desktop\\Career\\ETHZ\\ETHZ Work\\HardwareOutput\\Real-to-Sim-tests\\%.1f.csv", roundedIndex);
    data = readtable(filename);

    % Extract column 2 as strings
    rawForce = string(data{:, 2});  % NOT data(:,2) — use {} to get contents

    % Clean: remove 'kg' and whitespace
    cleanForce = str2double(erase(lower(strtrim(rawForce)), "kg"));

    % Get max
    maxVal = max(cleanForce);
    minVal=min(cleanForce);

    if abs(minVal)>maxVal
        loop_max=abs(minVal);
    else
        loop_max=maxVal
    end
    
    disp(loop_max);

    if loop_max>max_val
        disp(loop_max);
        max_val=loop_max;
    end
    

end
disp("The maximum value is: ");
disp(max_val);



%could rename all files to hertz an
