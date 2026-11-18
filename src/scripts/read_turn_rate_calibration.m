function capture = read_turn_rate_calibration(logPath)
% Read the last buffered Turn-rate calibration sequence in a monitor log.
text = fileread(logPath);
lines = splitlines(string(text));
starts = find(strtrim(lines) == "% CAL turn-rate-calibration");
required = [1, 3, 5:10, 13:15, 29:30, 46:62];
for start = flip(starts(:))'
    stop = numel(lines) + 1;
    for k = start+1:numel(lines)
        if startsWith(lines(k), "% CAL ") || startsWith(lines(k), "% Basebot log")
            stop = k;
            break;
        end
    end
    block = lines(start:stop-1);
    data = zeros(0, 62);
    phases = zeros(0, 10);
    for k = 1:numel(block)
        line = char(strtrim(block(k)));
        if startsWith(line, '% turn-phase ')
            values = sscanf(line(numel('% turn-phase ')+1:end), '%f')';
            if numel(values) == 10, phases(end+1,:) = values; end %#ok<AGROW>
        elseif ~isempty(line) && line(1) ~= '%'
            values = sscanf(line, '%f')';
            if numel(values) == 62 && all(isfinite(values(required)))
                data(end+1,:) = values; %#ok<AGROW>
            end
        end
    end
    if isempty(data), continue; end
    assert(all(diff(data(:,1)) > 0), 'Calibration timestamps must increase.');
    capture = struct('data', data, 'phases', phases, 'logPath', logPath);
    capture.text = char(join(block, newline));
    capture.mapping = headerVector(capture.text, '% Yaw mapping axis,sign,scale', [0,-1,1]);
    capture.offsets = headerVector(capture.text, '% Gyro offsets (library deg/s)', [0,0,0]);
    capture.gravity = headerVector(capture.text, '% Stationary gravity (library g)', [NaN,NaN,NaN]);
    result = regexp(capture.text, '(?m)^% Result: ([^\r\n]+)', 'tokens', 'once');
    capture.result = 'unknown';
    if ~isempty(result), capture.result = result{1}; end
    gear = headerNumber(capture.text, ', gear ([\d.eE+-]+)', 9.6);
    radius = headerNumber(capture.text, ', wheel radius ([\d.eE+-]+)', .0315);
    wheelBase = headerNumber(capture.text, 'm, wheel base ([\d.eE+-]+)', .14);
    ppr = headerNumber(capture.text, 'PPR (\d+)', 48);
    counts = data(:,44:45);
    delta = mod(diff(counts) + 32768, 65536) - 32768;
    distance = [zeros(1,2); cumsum(delta)] * (2*pi*radius/gear/ppr);
    capture.wheelAngle = (distance(:,2)-distance(:,1))/wheelBase;
    axis = capture.mapping(1)+1;
    factor = capture.mapping(2)*capture.mapping(3);
    capture.gyroAngle = data(:,59:61);
    capture.mappedAngle = factor * capture.gyroAngle(:,axis);
    capture.fusedAngle = unwrap(data(:,30)) - data(1,30);
    calibrated = bitand(uint16(data(:,3)), 2048) ~= 0;
    capture.conversionError = NaN;
    if any(calibrated)
        expected = -data(calibrated,56:58)*(1000/32768) - capture.offsets;
        capture.conversionError = max(abs(expected-data(calibrated,13:15)), [], 'all');
    end
    turns = phases(phases(:,2) ~= 0,:);
    duration = turns(:,4)-turns(:,3);
    gyro = factor*turns(:,4+axis);
    ratio = turns(:,8)./gyro;
    ratio(abs(gyro) < 1e-6) = NaN;
    capture.summary = table(turns(:,1), turns(:,2), duration, ...
        rad2deg(gyro), rad2deg(turns(:,8)), ratio, turns(:,9)./duration, ...
        100*turns(:,10), 'VariableNames', {'Phase','Reference_rad_s','Duration_s', ...
        'Gyro_deg','Wheel_deg','Wheel_over_gyro','Gyro_Hz','Saturation_percent'});
    capture.fit = NaN(3,2);
    wheelRate = turns(:,8)./duration;
    if height(capture.summary) >= 2 && max(wheelRate)-min(wheelRate) > 1e-6
        for axis = 1:3
            capture.fit(axis,:) = polyfit(turns(:,8)./duration, turns(:,4+axis)./duration, 1);
        end
    end
    return;
end
error('read_turn_rate_calibration:NoCapture', 'No valid turn-rate calibration capture in %s.', logPath);
end

function values = headerVector(text, prefix, fallback)
match = regexp(text, ['(?m)^', regexptranslate('escape',prefix), '\s+([^\r\n]+)'], 'tokens', 'once');
values = fallback;
if ~isempty(match), values = sscanf(match{1}, '%f')'; end
end

function value = headerNumber(text, expression, fallback)
match = regexp(text, expression, 'tokens', 'once');
value = fallback;
if ~isempty(match), value = str2double(match{1}); end
end
