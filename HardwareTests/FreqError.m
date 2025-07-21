%%measures the difference between real frequency and intended frequency of motor, as well as graphing and saving locally

%% Change user in directory
dataDir = "C:\Users\15405\OneDrive\Desktop\Career\ETHZ\ETHZ Work\HardwareOutput";

function [] = parse_loop(start_val, end_val, step)
%% USER SETTINGS: Toggle lines ON/OFF
showPosition     = true;
showGoal         = true;
showIdealGoal    = false;
showVelocity     = false;
showLoad         = false;
showAvgLoad      = true;  % New toggle for average load

for i=start_val:step:end_val
  %% Clear previous plots and settings
    close all;
    roundedIndex = round(i, 1);
    filename = sprintf("C:\\Users\\15405\\OneDrive\\Desktop\\Career\\ETHZ\\ETHZ Work\\HardwareOutput\\Servo_test_data_%.1f.csv", roundedIndex);
    data = readtable(filename);

    %% Read metadata (first two rows only)
    opts = detectImportOptions(filename);
    opts.VariableNamesLine = 3;
    opts.DataLines = [4 Inf];  % skip metadata
    data = readtable(filename, opts);

    fid = fopen(filename);
    meta_header = strsplit(fgetl(fid), ',');
    meta_values = str2double(strsplit(fgetl(fid), ','));
    fclose(fid);

    % Extract parameters
    desiredFreq  = meta_values(strcmp(meta_header, 'desiredFreq'));
    commandTime  = meta_values(strcmp(meta_header, 'commandTime'));
    stepSize     = meta_values(strcmp(meta_header, 'stepSize'));

    %% Extract data columns
    time          = data{:, 'Time_s_'};
    position      = 360/4096 * data{:, 'Position'};       % degrees
    goal_position = 360/4096 * data{:, 'GoalPosition'};   % degrees
    velocity      = 0.229 * data{:, 'Velocity'};           % RPM
    load_current  = 2.69 * data{:, 'Load'};                % mA

    % Ensure no NaNs in first goal value
    initialGoal = goal_position(1);
    if isnan(initialGoal)
        warning('goal_position(1) is NaN. Using position(1) instead.');
        initialGoal = position(1);
    end

    %% Generate ideal goal line
    ideal_time = (0:length(time)-1)' * commandTime;
    n = length(time);
    ideal_goal = zeros(n, 1);
    ideal_goal(1) = mod(initialGoal, 360);
    ideal_goal(2:end) = mod(initialGoal + (360/4096)*stepSize * (0:n-2)', 360);

    %% Estimate actual frequency using findpeaks
    gearRatio = 5.4;
    actualFreq = NaN;
    if showPosition
        [pks, locs] = findpeaks(mod(position, 360), time, 'MinPeakDistance', 1 / (2 * desiredFreq));
        if numel(locs) >= 2
            periods = diff(locs);
            actualFreq = gearRatio / mean(periods);
            fprintf('Estimated tail frequency from position peaks: %.3f Hz\n', actualFreq);
            fprintf('Instructed tail frequency: %.1f Hz\n', desiredFreq);
            error_val=(desiredFreq-actualFreq)/desiredFreq;
            fprintf('Error percentage: %.2f%%\n', error_val * 100);
        else
            fprintf('Not enough peaks to estimate frequency.\n');
        end
    end

    %% Plotting
    figure('Name', 'Servo Motion Data', 'Color', 'w');
    co = colororder(lines(6));  % Default color set
    colorIdx = 1;
    legendLabels = {};
    hold on;

    % Plot left Y-axis data
    yyaxis left
    if showPosition
        c = co(colorIdx,:); colorIdx = colorIdx + 1;
        plot(time, mod(position, 360), '-o', 'LineWidth', 1.5, ...
         'Color', c, 'MarkerSize', 4, 'MarkerFaceColor', c);
        legendLabels{end+1} = 'Position';
    end
    if showGoal
        c = co(colorIdx,:); colorIdx = colorIdx + 1;
        plot(time, mod(goal_position, 360), '-o', 'LineWidth', 1.5, ...
         'Color', c, 'MarkerSize', 4, 'MarkerFaceColor', c);
        legendLabels{end+1} = 'Goal Position';
    end
    if showIdealGoal
        c = co(colorIdx,:); colorIdx = colorIdx + 1;
        plot(time, ideal_goal, '-o', 'LineWidth', 1.5, ...
         'Color', c, 'MarkerSize', 4, 'MarkerFaceColor', c);
        legendLabels{end+1} = 'Ideal Goal';
    end
    ylabel('Position (degrees)');
    xlabel('Time (s)');
    grid on;

    % Plot right Y-axis ONLY if needed
    if showVelocity || showLoad || showAvgLoad
        yyaxis right
        if showVelocity
            c = co(colorIdx,:); colorIdx = colorIdx + 1;
            plot(time, velocity, '-o', 'LineWidth', 1.5, ...
             'Color', c, 'MarkerSize', 4, 'MarkerFaceColor', c);
            legendLabels{end+1} = 'Velocity';
        end
        if showLoad
            c = co(colorIdx,:); colorIdx = colorIdx + 1;
            plot(time, load_current, '-o', 'LineWidth', 1.5, ...
             'Color', c, 'MarkerSize', 4, 'MarkerFaceColor', c);
            legendLabels{end+1} = 'Load (Current)';
        end
        if showAvgLoad
            avgLoad = mean(load_current, 'omitnan');
            c = co(colorIdx,:); colorIdx = colorIdx + 1;
            plot(time, repmat(avgLoad, size(time)), '--', 'LineWidth', 2, ...
             'Color', c);
            legendLabels{end+1} = sprintf('Avg Load = %.1f mA', avgLoad);
        end
        ylabel('Velocity / Load (RPM / mA)');
    else
        yyaxis right
        ax = gca;
        ax.YAxis(2).Visible = 'off';
    end

    % Format timestamp from filename for title
    [~, baseFileName, ~] = fileparts(filename);
    timestampStr = regexprep(baseFileName, 'servo_test_data.*', '');
    timestampStr = strrep(timestampStr, '_', ' ');
    %readableTime = datetime(timestampStr, 'InputFormat', 'yyyyMMdd HHmm', 'Format', 'yyyy-MM-dd HH:mm');

    % Format title
    if ~isnan(actualFreq)
        plotTitle = sprintf('%s | f = %.1f Hz (cmd), %.2f Hz (meas)', ...
                         desiredFreq, actualFreq);
    else
        plotTitle = sprintf('%s | f = %.1f Hz (cmd)', ...
                         desiredFreq);
    end
    title(plotTitle, 'Interpreter', 'none');

    % Final plot formatting
    legend(legendLabels, 'Location', 'best');
    ax = gca;
    ax.XAxis.Exponent = 0;
    ax.YAxis(1).Exponent = 0;
    if showVelocity || showLoad || showAvgLoad
        ax.YAxis(2).Exponent = 0;
    end
    hold off;

    %% Energetics
    voltage = 12;
    if showAvgLoad
        fprintf('Average Current: %.3f mA\n', avgLoad);
        fprintf('Max Instantaneous Current: %.3f mA\n', max(load_current));
        avgPower = voltage * avgLoad / 1000;
        maxPower = voltage * max(load_current) / 1000;
        fprintf('Average Power: %.3f W\n', avgPower);
        fprintf('Max Instantaneous Power: %.3f W\n', maxPower);
    end

    % Format title properly
    if ~isnan(actualFreq)
        plotTitle = sprintf('f = %.1f Hz (cmd), %.2f Hz (meas)', desiredFreq, actualFreq);
    else
        plotTitle = sprintf('f = %.1f Hz (cmd)', desiredFreq);
    end
    title(plotTitle, 'Interpreter', 'none');

    % Sanitize filename
    safePlotTitle = regexprep(plotTitle, '[\/:*?"<>|,=()]', '_');
    safePlotTitle = strrep(safePlotTitle, ' ', '_');

    % Create folder if it doesn't exist
    folder = 'C:\Users\15405\OneDrive\Desktop\Career\ETHZ\ETHZ Work\HardwareOutput\MatlabPlots';
    if ~exist(folder, 'dir')
        mkdir(folder);
    end

    % Save figure
    filenameFig = fullfile(folder, [safePlotTitle, '.fig']);
    saveas(gcf, filenameFig);

    %% Vel Mode Slope Calcs
    x1 = 6.116; %seconds
    y1 = 52821; %ticks
    x2 = 13.698; %seconds
    y2 = 73735; %ticks
    slope = (y2-y1)/(x2-x1); %ticks/second
    motorTicks = 4096; %ticks/rev
    avgFreq = gearRatio*slope/4096; %Hz
    fprintf('Average Frequency: %.3f Hz\n', avgFreq);
end
end

parse_loop(1,2,.1);
parse_loop(2.2,4,.2);


