%% run_pick_and_place.m
% =========================================================================
%   5-DOF ROBOT ARM: PICK-AND-PLACE MISSION SIMULATION & CONTROL BENCHMARK
% =========================================================================
% Simulates a realistic industrial pick-and-place operation:
%   Phase 1 (0.0 - 2.0s):  Approach Pick Station A (Hover)
%   Phase 2 (2.0 - 3.5s):  Descend onto Object at Point A
%   Phase 3 (3.5 - 4.5s):  Grasp Object at Point A (Dwell)
%   Phase 4 (4.5 - 6.0s):  Lift Object to Clearance Height
%   Phase 5 (6.0 - 8.5s):  Transit across workspace to Place Station B (Hover)
%   Phase 6 (8.5 - 10.0s): Descend onto drop fixture at Point B
%   Phase 7 (10.0 - 11.0s): Release Object at Point B (Dwell)
%   Phase 8 (11.0 - 12.0s): Retract / Lift from Point B
%   Phase 9 (12.0 - 14.0s): Return to Safe Home / Standby Pose
%
% Controllers:
%   - Computed Torque Control (CTC): Feedback linearization + PD
%   - Joint-space PID: Decoupled PID
% =========================================================================

projectDir = fileparts(mfilename('fullpath'));
if isempty(projectDir), projectDir = pwd; end
cd(projectDir);

fprintf('========================================================================\n');
fprintf('        5-DOF ROBOT ARM: PICK-AND-PLACE SIMULATION & BENCHMARK          \n');
fprintf('========================================================================\n\n');

model_nominal = 'FiveDOF_Robot_Arm';
model_ctc = 'FiveDOF_Robot_Arm_CTC';

% Load robot dynamics model
load_system(model_nominal);
global robot_nominal;
robot_nominal = importrobot(model_nominal);
robot_nominal.DataFormat = 'column';
robot_nominal.Gravity = [0; 0; -9.81];
ee_name = robot_nominal.BodyNames{end};
robot = robot_nominal;

% Load Simulink model
load_system(model_ctc);
set_param(model_ctc, 'SimMechanicsOpenEditorOnUpdate', 'off');
set_param(model_ctc, 'EnablePacing', 'off');

% Simulation timing
t_sim_stop = 14.0;
set_param(model_ctc, 'StopTime', num2str(t_sim_stop));

% Define test conditions:
% Compare CTC vs PID on the Pick-and-Place mission carrying a 0.5 kg object
payload_mass = 0.5; % 0.5 kg picked object
set_param([model_ctc '/Payload_Solid'], 'Mass', 'payload_mass');
assignin('base', 'payload_mass', payload_mass);

% PID & CTC Controller Gains
global Kp_pid Kd_pid Ki_pid Kp_ctc Kd_ctc;
Kp_pid = [350; 600; 450; 250; 80];
Kd_pid = [45; 75; 55; 30; 10];
Ki_pid = [25; 45; 35; 20; 5];
Kp_ctc = [225; 225; 225; 225; 225];
Kd_ctc = [30; 30; 30; 30; 30];

assignin('base', 'Kp_pid', Kp_pid);
assignin('base', 'Kd_pid', Kd_pid);
assignin('base', 'Ki_pid', Ki_pid);
assignin('base', 'Kp_ctc', Kp_ctc);
assignin('base', 'Kd_ctc', Kd_ctc);

controllers = {'CTC', 'PID'};
ctrl_modes = [2, 1];
pnp_results = struct();

fprintf('--- Running Pick-and-Place Simulations (Payload = %.1f kg) ---\n', payload_mass);
for c = 1:length(controllers)
    cName = controllers{c};
    cMode = ctrl_modes(c);
    fprintf('Running %s controller ... ', cName);
    
    % Reset controller state
    clear robot_controller_fcn;
    global prev_t e_int;
    prev_t = [];
    e_int = zeros(5, 1);
    
    set_param([model_ctc '/ctrl_mode'], 'Value', num2str(cMode));
    assignin('base', 'ctrl_mode', cMode);
    
    simOut = sim(model_ctc, 'StopTime', num2str(t_sim_stop));
    
    t = simOut.q_out.Time;
    q = simOut.q_out.Data;
    qd = simOut.qd_out.Data;
    traj = simOut.traj_ref.Data;
    tau = simOut.tau_out.Data;
    
    qd_ref = traj(:, 1:5);
    qdot_ref = traj(:, 6:10);
    e = qd_ref - q;
    
    % Compute Forward Kinematics along trajectory
    N_pts = length(t);
    P_actual = zeros(N_pts, 3);
    P_ref = zeros(N_pts, 3);
    
    for i = 1:N_pts
        T_act = getTransform(robot, q(i, :)', ee_name);
        P_actual(i, :) = T_act(1:3, 4)';
        
        T_ref = getTransform(robot, qd_ref(i, :)', ee_name);
        P_ref(i, :) = T_ref(1:3, 4)';
    end
    
    err_cart = sqrt(sum((P_ref - P_actual).^2, 2)); % meters
    
    % Store
    pnp_results(c).Controller = cName;
    pnp_results(c).Time = t;
    pnp_results(c).q = q;
    pnp_results(c).qd = qd;
    pnp_results(c).q_ref = qd_ref;
    pnp_results(c).qd_ref = qdot_ref;
    pnp_results(c).e = e;
    pnp_results(c).tau = tau;
    pnp_results(c).P_actual = P_actual;
    pnp_results(c).P_ref = P_ref;
    pnp_results(c).err_cart = err_cart;
    pnp_results(c).RMS_Cart_mm = sqrt(mean(err_cart.^2)) * 1000;
    pnp_results(c).Max_Cart_mm = max(err_cart) * 1000;
    pnp_results(c).RMS_Joint_rad = sqrt(mean(e.^2, 1));
    pnp_results(c).Peak_Tau_Nm = max(abs(tau), [], 1);
    
    fprintf('Done. (Cartesian RMS: %.2f mm, Max: %.2f mm)\n', ...
        pnp_results(c).RMS_Cart_mm, pnp_results(c).Max_Cart_mm);
end
fprintf('\nAll Pick-and-Place simulations finished.\n\n');

%% Save MAT and CSV
outDir = projectDir;
save(fullfile(outDir, 'pick_and_place_results.mat'), 'pnp_results');

summary_pnp = cell(2, 6);
for c = 1:2
    summary_pnp(c, :) = {
        pnp_results(c).Controller, ...
        payload_mass, ...
        pnp_results(c).RMS_Cart_mm, ...
        pnp_results(c).Max_Cart_mm, ...
        norm(pnp_results(c).RMS_Joint_rad), ...
        max(pnp_results(c).Peak_Tau_Nm)
    };
end
pnp_table = cell2table(summary_pnp, 'VariableNames', { ...
    'Controller', 'Payload_kg', 'RMS_Cartesian_mm', 'Max_Cartesian_mm', ...
    'Norm_RMS_Joint_rad', 'Max_Peak_Torque_Nm' ...
});
writetable(pnp_table, fullfile(outDir, 'pick_and_place_results.csv'));
disp(pnp_table);

%% PLOTTING SUITE (Pure White, Publication Quality)
set(0, 'DefaultAxesFontName', 'Arial', 'DefaultAxesFontSize', 9);
set(0, 'DefaultLineLineWidth', 1.5);

c_ctc = [0.0, 0.45, 0.74]; % Blue
c_pid = [0.85, 0.33, 0.10]; % Red/Orange
c_ref = [0.15, 0.15, 0.15]; % Dark Gray

p_Apick = pnp_results(1).P_ref(find(pnp_results(1).Time >= 3.5, 1), :);
p_Bdrop = pnp_results(1).P_ref(find(pnp_results(1).Time >= 10.0, 1), :);

% FIGURE 1: 3D Trajectory in Workspace
fig1 = figure('Name', '3D Trajectory', 'Position', [100, 100, 900, 700], 'Color', 'w', 'Visible', 'off');
plot3(pnp_results(1).P_ref(:, 1), pnp_results(1).P_ref(:, 2), pnp_results(1).P_ref(:, 3), ...
    'k--', 'LineWidth', 2.0, 'DisplayName', 'Reference Path'); hold on;
plot3(pnp_results(1).P_actual(:, 1), pnp_results(1).P_actual(:, 2), pnp_results(1).P_actual(:, 3), ...
    '-', 'Color', c_ctc, 'LineWidth', 2.0, 'DisplayName', 'CTC Actual Path');
plot3(pnp_results(2).P_actual(:, 1), pnp_results(2).P_actual(:, 2), pnp_results(2).P_actual(:, 3), ...
    '-.', 'Color', c_pid, 'LineWidth', 1.8, 'DisplayName', 'PID Actual Path');

% Plot Pick and Place Stations
plot3(p_Apick(1), p_Apick(2), p_Apick(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', [0.2 0.8 0.2], 'DisplayName', 'Point A (Pick Station)');
plot3(p_Bdrop(1), p_Bdrop(2), p_Bdrop(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', [0.9 0.2 0.2], 'DisplayName', 'Point B (Place Station)');

text(p_Apick(1)+0.03, p_Apick(2), p_Apick(3)+0.02, 'Point A (Pick)', 'FontWeight', 'bold', 'Color', [0 0.5 0]);
text(p_Bdrop(1)+0.03, p_Bdrop(2), p_Bdrop(3)+0.02, 'Point B (Place)', 'FontWeight', 'bold', 'Color', [0.8 0 0]);

grid on; box on;
xlabel('X Position (m)'); ylabel('Y Position (m)'); zlabel('Z Position (m)');
title({'5-DOF Robot Arm: 3D End-Effector Pick-and-Place Trajectory', ...
       sprintf('Pick Station A \\rightarrow Place Station B (Payload = %.1f kg)', payload_mass)}, ...
       'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'northeast');
view(45, 25);
exportgraphics(fig1, fullfile(outDir, 'pick_and_place_3d_trajectory.png'), 'Resolution', 300);
close(fig1);
fprintf('Saved pick_and_place_3d_trajectory.png (300 DPI)\n');

% FIGURE 2: Cartesian Coordinates vs Time (X, Y, Z, Error)
fig2 = figure('Name', 'Cartesian Tracking', 'Position', [100, 100, 1100, 800], 'Color', 'w', 'Visible', 'off');
coords = {'X', 'Y', 'Z'};
for i = 1:3
    subplot(2, 2, i); hold on; grid on; box on;
    plot(pnp_results(1).Time, pnp_results(1).P_ref(:, i), 'k--', 'LineWidth', 1.8, 'DisplayName', 'Desired');
    plot(pnp_results(1).Time, pnp_results(1).P_actual(:, i), '-', 'Color', c_ctc, 'LineWidth', 1.6, 'DisplayName', 'CTC');
    plot(pnp_results(2).Time, pnp_results(2).P_actual(:, i), '-.', 'Color', c_pid, 'LineWidth', 1.4, 'DisplayName', 'PID');
    xlabel('Time (s)'); ylabel(sprintf('%s Position (m)', coords{i}));
    title(sprintf('Tool Center Point %s-Coordinate Tracking', coords{i}), 'FontWeight', 'bold');
    if i == 1, legend('Location', 'best'); end
end

subplot(2, 2, 4); hold on; grid on; box on;
plot(pnp_results(1).Time, pnp_results(1).err_cart * 1000, '-', 'Color', c_ctc, 'LineWidth', 1.8, ...
    'DisplayName', sprintf('CTC (RMS = %.1f mm)', pnp_results(1).RMS_Cart_mm));
plot(pnp_results(2).Time, pnp_results(2).err_cart * 1000, '-.', 'Color', c_pid, 'LineWidth', 1.5, ...
    'DisplayName', sprintf('PID (RMS = %.1f mm)', pnp_results(2).RMS_Cart_mm));
xlabel('Time (s)'); ylabel('3D Euclidean Error (mm)');
title('Cartesian End-Effector Tracking Error e_{cart}(t)', 'FontWeight', 'bold');
legend('Location', 'northeast');

sgtitle('End-Effector Cartesian Kinematics across 9 Pick-and-Place Mission Phases', ...
    'FontSize', 12, 'FontWeight', 'bold');
exportgraphics(fig2, fullfile(outDir, 'pick_and_place_cartesian_xyz.png'), 'Resolution', 300);
close(fig2);
fprintf('Saved pick_and_place_cartesian_xyz.png (300 DPI)\n');

% FIGURE 3: Joint Space Tracking (5 Subplots)
fig3 = figure('Name', 'Joint Tracking', 'Position', [50, 50, 1200, 900], 'Color', 'w', 'Visible', 'off');
joint_names = {'Joint 1 (Waist Yaw)', 'Joint 2 (Shoulder Pitch)', 'Joint 3 (Elbow Pitch)', ...
               'Joint 4 (Wrist Pitch)', 'Joint 5 (Wrist Roll)'};
for j = 1:5
    subplot(3, 2, j); hold on; grid on; box on;
    plot(pnp_results(1).Time, pnp_results(1).q_ref(:, j), 'k--', 'LineWidth', 1.8, 'DisplayName', 'Reference');
    plot(pnp_results(1).Time, pnp_results(1).q(:, j), '-', 'Color', c_ctc, 'LineWidth', 1.6, 'DisplayName', 'CTC');
    plot(pnp_results(2).Time, pnp_results(2).q(:, j), '-.', 'Color', c_pid, 'LineWidth', 1.4, 'DisplayName', 'PID');
    xlabel('Time (s)'); ylabel('Joint Angle (rad)');
    title(joint_names{j}, 'FontWeight', 'bold');
    if j == 1, legend('Location', 'northeast', 'FontSize', 8); end
end
sgtitle('5-DOF Joint Angles Tracking: CTC vs Decoupled PID', 'FontSize', 12, 'FontWeight', 'bold');
exportgraphics(fig3, fullfile(outDir, 'pick_and_place_tracking.png'), 'Resolution', 300);
close(fig3);
fprintf('Saved pick_and_place_tracking.png (300 DPI)\n');

% FIGURE 4: Actuator Torques (5 Subplots)
fig4 = figure('Name', 'Joint Torques', 'Position', [50, 50, 1200, 900], 'Color', 'w', 'Visible', 'off');
for j = 1:5
    subplot(3, 2, j); hold on; grid on; box on;
    plot(pnp_results(1).Time, pnp_results(1).tau(:, j), '-', 'Color', c_ctc, 'LineWidth', 1.6, 'DisplayName', 'CTC Torque');
    plot(pnp_results(2).Time, pnp_results(2).tau(:, j), '-.', 'Color', c_pid, 'LineWidth', 1.4, 'DisplayName', 'PID Torque');
    xlabel('Time (s)'); ylabel('Torque (N\cdotm)');
    title(joint_names{j}, 'FontWeight', 'bold');
    if j == 1, legend('Location', 'northeast', 'FontSize', 8); end
end
sgtitle('Actuator Control Torques Across Pick-and-Place Mission Phases', 'FontSize', 12, 'FontWeight', 'bold');
exportgraphics(fig4, fullfile(outDir, 'pick_and_place_torques.png'), 'Resolution', 300);
close(fig4);
fprintf('Saved pick_and_place_torques.png (300 DPI)\n');

close_system(model_nominal, 0);
close_system(model_ctc, 0);
fprintf('Pick-and-place execution, tables, and figures completed!\n');
