% record_mechanics_explorer_video.m
% =========================================================================
%   RECORD 5-DOF ROBOT SIMULATION VIDEO FROM SIMSCAPE MECHANICS EXPLORER
% =========================================================================
% This script runs the Computed Torque Control (CTC) Pick-and-Place
% simulation in MATLAB/Simulink and exports the studio-grade 3D CAD
% animation from Mechanics Explorer into FiveDOF_Robot_Arm_CTC.mp4
% (matching ThreeDOF_Robot_Arm_CTC.mp4 format).
% =========================================================================

projectDir = fileparts(mfilename('fullpath'));
if isempty(projectDir), projectDir = pwd; end
cd(projectDir);

modelName = 'FiveDOF_Robot_Arm_CTC';
videoName = 'FiveDOF_Robot_Arm_CTC.mp4';

fprintf('========================================================================\n');
fprintf('     5-DOF ROBOT ARM: RECORDING MECHANICS EXPLORER SIMULATION VIDEO     \n');
fprintf('========================================================================\n\n');

fprintf('Opening %s in Simulink...\n', modelName);
open_system(modelName);

% Set simulation parameters
set_param(modelName, 'SimMechanicsOpenEditorOnUpdate', 'on');
set_param(modelName, 'StopTime', '14.0');
assignin('base', 'ctrl_mode', 2);       % 2 = Computed Torque Control (CTC)
assignin('base', 'payload_mass', 0.5);   % 0.5 kg Pick-and-Place payload

fprintf('Starting dynamic simulation in Simulink to populate Mechanics Explorer...\n');
set_param(modelName, 'SimulationCommand', 'start');

% Wait for the simulation to finish
fprintf('Simulating 14-second Pick-and-Place mission... Please wait.\n');
while ~strcmp(get_param(modelName, 'SimulationStatus'), 'stopped')
    pause(0.5);
end
fprintf('Simulation complete! Mechanics Explorer is loaded with trajectory.\n\n');

% Export video directly from Mechanics Explorer
fprintf('Recording video via smwritevideo to: %s\n', videoName);
try
    smwritevideo(modelName, videoName, ...
        'VideoFormat', 'mpeg-4', ...
        'FrameSize', [2400, 1350], ...
        'FrameRate', 60, ...
        'PlaybackSpeedRatio', 1.0);
    
    fprintf('\nSUCCESS! Video recorded and saved to:\n  %s\n', ...
        fullfile(pwd, videoName));
catch ME
    fprintf('Note: Could not record automatically via script: %s\n\n', ME.message);
    fprintf('You can record it in 2 clicks directly from Mechanics Explorer:\n');
    fprintf('  1. In the Mechanics Explorer window, look at the top ribbon toolbar under "APPS".\n');
    fprintf('  2. Click the "Video Creator" button (green filmstrip/movie slate icon).\n');
    fprintf('  3. In Video Creator dialog:\n');
    fprintf('     - Video File Name: FiveDOF_Robot_Arm_CTC.mp4\n');
    fprintf('     - Video Format:    MPEG-4\n');
    fprintf('     - Frame Size:      Custom [2400 1350] (or 1920 x 1080)\n');
    fprintf('     - Frame Rate:      60 fps\n');
    fprintf('  4. Click "Create Video" (or Record).\n');
end
