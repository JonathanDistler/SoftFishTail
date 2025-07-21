%measures the force in the direction of the fish's head
clear length
for index = 2:0.2:4  % Use colon syntax correctly
    figure;  % Opens a new figure each loop

    % Round index to 1 decimal to avoid floating point precision issues
    roundedIndex = round(index, 1);

    % Format CV tracking filename
    file = "C:\\Users\\15405\\OneDrive\\Desktop\\Career\\ETHZ\\ETHZ Work\\HardwareOutput\\Live-Tests\\CV_Tail_Track\\HeadSegement_rel_Stationary_%.1f.csv";
    filename = sprintf(file, roundedIndex);
    data = readtable(filename);

    % Format force data filename
    forceFile = "C:\\Users\\15405\\OneDrive\\Desktop\\Career\\ETHZ\\ETHZ Work\\HardwareOutput\\Live-Tests\\%.1f.csv";
    forcefilename = sprintf(forceFile, roundedIndex);
    forceData = readtable(forcefilename);

    %force in the directino of the thrust sensor
    % Extract column 2 as strings, force, row 2 to match with the row for
    % time, and to avoid NaN in metaData
    %with the changed CV file, rawAngle will end up being 3. . . 4, instead
    %of 3.. . 6
    rawAngle = (data{3:end, 6});
    lengthAngle=length(rawAngle);
    
  
    sineAngle=sind(rawAngle);

    rawForce = string(forceData{3:end, 2});
    % Clean: remove 'kg' and whitespace
    cleanForce = str2double(erase(lower(strtrim(rawForce)), "kg"));
    lengthForce=length(cleanForce);

    %brings in the time value from the force-value csv rather than relying
    %on an FPS conversion like previously
    rawTime=(forceData{3:end,1})

    forwardForce=-1*cleanForce.*sineAngle

    %plots force vs time for each respective hertz
    plot(rawTime,forwardForce)
    yline(0,"r")
    title(sprintf('%.1f Hz: Time vs Forward Force', roundedIndex));
    xlabel('Time (s)');
    ylabel('Force (kg)');
    % Save figure to its respective folder
    folder = 'C:\Users\15405\OneDrive\Desktop\Career\ETHZ\ETHZ Work\HardwareOutput\Live-Tests\Graphs';
    filenameFig = fullfile(folder, sprintf('%.1f_HZ_TimevsFowardForce.fig', roundedIndex));
    saveas(gcf, filenameFig);
    
  
end

