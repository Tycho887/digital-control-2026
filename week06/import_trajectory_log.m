function experiment = import_trajectory_log(logPath)
% Import a serial capture without altering saved robot/filter models.
% experiment = import_trajectory_log('src/logs/trajectory-20261018.log');
text=fileread(logPath); lines=splitlines(string(text)); rows={};
for k=1:numel(lines)
    line=strtrim(lines(k));
    if strlength(line)==0 || startsWith(line,'%'), continue; end
    values=sscanf(char(line),'%f')';
    if numel(values)==55, rows{end+1,1}=values; end %#ok<AGROW>
end
assert(~isempty(rows),'No 55-column trajectory rows found.');
data=vertcat(rows{:});
time=data(:,17)*1e-6;
assert(all(diff(time)>0),'Capture timestamps must increase; import one run at a time.');
experiment=struct('source',logPath,'headers',lines(startsWith(strtrim(lines),'%')), ...
    'data',data,'time',time,'phase',data(:,2),'commands',data(:,45:46), ...
    'motorSpeed',data(:,8:9),'motorVoltage',data(:,4:5), ...
    'encoderCounts',data(:,6:7),'encoderDelay',data(:,23:24), ...
    'state',data(:,25:32),'variance',data(:,33:40),'position',data(:,41:42), ...
    'imuUsed',data(:,43:44),'estimatorTime',data(:,47)*1e-6, ...
    'runtimeUs',data(:,48),'flags',uint8(data(:,49)), ...
    'accelG',data(:,50:52),'gyroDegreesPerSecond',data(:,53:55));
% Directly usable held-command schedule, including final stationary samples.
changes=[true;any(diff(experiment.commands),2)];
experiment.commandSchedule=[time(changes),experiment.commands(changes,:)];
experiment.sampleIntervals=diff(time);
[folder,name]=fileparts(logPath);
save(fullfile(folder,[name '-experiment.mat']),'experiment');
fprintf('Imported %d rows, %.3f s; max estimator runtime %.0f us\n', ...
    size(data,1),time(end)-time(1),max(experiment.runtimeUs));
end
