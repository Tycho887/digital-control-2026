function selected = tune_basebot_p()
% P-only pidtune sweep, validated in the nonlinear sampled Simulink model.
% Objective: minimum integral absolute speed error among candidates with
% <2% steady error, settled response, and <15% reference overshoot.
folder=fileparts(mfilename('fullpath'));
model='week06_basebot_test';
load_system(fullfile(folder,[model '.slx']));
workspace=get_param(model,'ModelWorkspace');
motorParameters=workspace.evalin('wheels');
rows=table();
frequencies=logspace(log10(20),log10(1000),24); % rad/s crossover requests
options=pidtuneOptions('PhaseMargin',60);
print