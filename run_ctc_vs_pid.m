%% Master Script: CTC vs PID Dynamics and Control Comparison
% Compares Computed Torque Control (CTC) vs Decoupled Joint PID on 5-DOF Simscape Multibody robot arm
% under payload variations (0 kg, 0.5 kg, 1.0 kg).

projectDir = fileparts(mfilename('fullpath'));
if isempty(projectDir), projectDir = pwd; end
cd(projectDir);

fprintf('========================================================================\n');
fprintf('  5-DOF ROBOT ARM: COMPUTED TORQUE CONTROL (CTC) VS JOINT PID BENCHMARK \n');
fprintf('========================================================================\n\n');

%% STEP 1 & 2: Load Model and Rigid Body Tree Dynamics
model_nominal = 'FiveDOF_Robot_Arm';
model_ctc = 'FiveDOF_Robot_Arm_CTC';

load_system(model_nominal);
global robot_nominal;
robot_nominal = importrobot(model_nominal);
robot_nominal.DataFormat = 'column';
robot_nominal.Gravity = [0; 0; -9.81];

fprintf('Dynamics Model (rigidBodyTree) successfully imported.\n');
fprintf('Total rigid bodies: %d\n', robot_nominal.NumBodies);
for bIdx = 1:robot_nominal.NumBodies
    bName = robot_nominal.BodyNames{bIdx};
    fprintf('  - Body %d: %-15s (Mass: %.4f kg)\n', bIdx, bName, robot_nominal.Bodies{bIdx}.Mass);
end
fprintf('\n');

%% STEP 4 & 5: Controller Setup and Simulation Parameters
load_system(model_ctc);
set_param(model_ctc, 'SimMechanicsOpenEditorOnUpdate', 'off');
set_param(model_ctc, 'EnablePacing', 'off');

% Controller A: Decoupled Joint PID (calibrated for nominal 0 kg payload)
global Kp_pid Kd_pid Ki_pid;
Kp_pid = [350; 600; 450; 250; 80];
Kd_pid = [45; 75; 55; 30; 10];
Ki_pid = [25; 45; 35; 20; 5];

% Controller B: CTC (critically damped: omega_n = 15 rad/s -> Kp = 225, Kd = 30)
global Kp_ctc Kd_ctc;
Kp_ctc = [225; 225; 225; 225; 225];
Kd_ctc = [30; 30; 30; 30; 30];

payloads = [0.0, 0.5, 1.0];
controllers = {'PID', 'CTC'};
ctrl_modes = [1, 2];

results = struct();
rec_idx = 1;

fprintf('--- STEP 5: Running Simulation Experiments (6 Test Cases) ---\n');
for c_idx = 1:length(controllers)
    ctrl_name = controllers{c_idx};
    ctrl_mode = ctrl_modes(c_idx);
    
    for p_idx = 1:length(payloads)
        payload_mass = payloads(p_idx);
        fprintf('Running: Controller = %-3s | Payload = %.1f kg ... ', ctrl_name, payload_mass);
        
        % Reset controller persistent variables
        clear robot_controller_fcn;
        global prev_t e_int;
        prev_t = [];
        e_int = zeros(5, 1);
        
        payload_mass_val = max(payload_mass, 1e-4);
        set_param([model_ctc '/Payload_Solid'], 'Mass', 'payload_mass');
        set_param([model_ctc '/ctrl_mode'], 'Value', num2str(ctrl_mode));
        
        assignin('base', 'payload_mass', payload_mass_val);
        assignin('base', 'ctrl_mode', ctrl_mode);
        assignin('base', 'Kp_pid', Kp_pid);
        assignin('base', 'Kd_pid', Kd_pid);
        assignin('base', 'Ki_pid', Ki_pid);
        assignin('base', 'Kp_ctc', Kp_ctc);
        assignin('base', 'Kd_ctc', Kd_ctc);
        
        simOut = sim(model_ctc, 'StopTime', '8.0');
        
        t = simOut.q_out.Time;
        q = simOut.q_out.Data;
        qd = simOut.qd_out.Data;
        traj = simOut.traj_ref.Data;
        tau = simOut.tau_out.Data;
        
        % Reference q_d and tracking error (5 joints)
        qd_ref = traj(:, 1:5);
        e = qd_ref - q;
        
        % Performance Metrics per joint
        rms_err = sqrt(mean(e.^2, 1));
        max_err = max(abs(e), [], 1);
        peak_tau = max(abs(tau), [], 1);
        
        % Settling time at final target w2 (holding after t=6.0s until 8.0s)
        t_hold_idx = find(t >= 6.0);
        settling_time = zeros(1, 5);
        for j = 1:5
            tol = 0.02; % 0.02 rad threshold
            outside_idx = find(abs(e(t_hold_idx, j)) > tol);
            if isempty(outside_idx)
                settling_time(j) = 6.0;
            else
                settling_time(j) = t(t_hold_idx(outside_idx(end)));
            end
        end
        
        % Store results
        results(rec_idx).Controller = ctrl_name;
        results(rec_idx).Payload_kg = payload_mass;
        results(rec_idx).Time = t;
        results(rec_idx).q = q;
        results(rec_idx).qd = qd;
        results(rec_idx).q_ref = qd_ref;
        results(rec_idx).e = e;
        results(rec_idx).tau = tau;
        results(rec_idx).RMS = rms_err;
        results(rec_idx).MaxErr = max_err;
        results(rec_idx).PeakTau = peak_tau;
        results(rec_idx).Settling = settling_time;
        
        for j = 1:5
            results(rec_idx).(sprintf('RMS_J%d', j)) = rms_err(j);
            results(rec_idx).(sprintf('MaxErr_J%d', j)) = max_err(j);
            results(rec_idx).(sprintf('PeakTau_J%d', j)) = peak_tau(j);
            results(rec_idx).(sprintf('Settling_J%d', j)) = settling_time(j);
        end
        
        fprintf('Done. (RMS Err J1-5: [%.4f, %.4f, %.4f, %.4f, %.4f] rad)\n', ...
            rms_err(1), rms_err(2), rms_err(3), rms_err(4), rms_err(5));
        rec_idx = rec_idx + 1;
    end
end
fprintf('\nAll simulations completed successfully.\n\n');

%% STEP 6: Save Results Table and Data
fprintf('--- STEP 6: Saving Results and Generating Figures ---\n');
outDir = projectDir;

varNames = {'Controller', 'Payload_kg'};
for j = 1:5, varNames{end+1} = sprintf('RMS_J%d_rad', j); end
for j = 1:5, varNames{end+1} = sprintf('MaxErr_J%d_rad', j); end
for j = 1:5, varNames{end+1} = sprintf('PeakTau_J%d_Nm', j); end
for j = 1:5, varNames{end+1} = sprintf('Settling_J%d_s', j); end

summary_data = cell(length(results), length(varNames));
for r = 1:length(results)
    row = {results(r).Controller, results(r).Payload_kg};
    for j = 1:5, row{end+1} = results(r).(sprintf('RMS_J%d', j)); end
    for j = 1:5, row{end+1} = results(r).(sprintf('MaxErr_J%d', j)); end
    for j = 1:5, row{end+1} = results(r).(sprintf('PeakTau_J%d', j)); end
    for j = 1:5, row{end+1} = results(r).(sprintf('Settling_J%d', j)); end
    summary_data(r, :) = row;
end

results_table = cell2table(summary_data, 'VariableNames', varNames);

fprintf('\n===============================================================================================================\n');
fprintf('                                          BENCHMARK RESULTS TABLE                                              \n');
fprintf('===============================================================================================================\n');
disp(results_table);

% Save to CSV and MAT
writetable(results_table, fullfile(outDir, 'results_ctc_vs_pid.csv'));
save(fullfile(outDir, 'results_ctc_vs_pid.mat'), 'results', 'results_table');
fprintf('Saved results to results_ctc_vs_pid.csv and results_ctc_vs_pid.mat\n\n');

%% STEP 7: Publication-Quality Figures (Pure White Background, Crisp Black Labels)
set(0, 'DefaultAxesFontName', 'Arial', 'DefaultAxesFontSize', 9);
set(0, 'DefaultLineLineWidth', 1.5);

colors = [0.0, 0.45, 0.74;  % blue
          0.85, 0.33, 0.10;  % orange
          0.93, 0.69, 0.13;  % yellow
          0.49, 0.18, 0.56;  % purple
          0.47, 0.67, 0.19;  % green
          0.64, 0.08, 0.18]; % red

styles = {'-', '--', ':', '-.', '-', '--'};
joint_names = {'Joint 1 (Waist Yaw)', 'Joint 2 (Shoulder Pitch)', 'Joint 3 (Elbow Pitch)', ...
               'Joint 4 (Wrist Pitch)', 'Joint 5 (Wrist Roll)'};

% FIGURE 1: Tracking Error vs Time (5 subplots)
fig1 = figure('Name', 'Tracking Error vs Time', 'Position', [50, 50, 1200, 900], ...
              'Color', 'w', 'Visible', 'off');
for j = 1:5
    subplot(3, 2, j); hold on; grid on; box on;
    for r = 1:length(results)
        lbl = sprintf('%s (m=%.1f kg)', results(r).Controller, results(r).Payload_kg);
        plot(results(r).Time, results(r).e(:, j), ...
            'Color', colors(r, :), 'LineStyle', styles{r}, 'DisplayName', lbl);
    end
    xlabel('Time (s)'); ylabel('Error (rad)');
    title(joint_names{j}, 'FontWeight', 'bold');
    if j == 1, legend('Location', 'northeast', 'FontSize', 7); end
end
sgtitle('5-DOF Tracking Error vs Time: Computed Torque Control vs Decoupled PID', ...
    'FontSize', 12, 'FontWeight', 'bold');

saveas(fig1, fullfile(outDir, 'tracking_error_vs_time.fig'));
exportgraphics(fig1, fullfile(outDir, 'tracking_error_vs_time.png'), 'Resolution', 300);
close(fig1);
fprintf('Saved tracking_error_vs_time.png (300 DPI)\n');

% FIGURE 2: Joint Torques vs Time (5 subplots)
fig2 = figure('Name', 'Joint Torques vs Time', 'Position', [50, 50, 1200, 900], ...
              'Color', 'w', 'Visible', 'off');
for j = 1:5
    subplot(3, 2, j); hold on; grid on; box on;
    for r = 1:length(results)
        lbl = sprintf('%s (m=%.1f kg)', results(r).Controller, results(r).Payload_kg);
        plot(results(r).Time, results(r).tau(:, j), ...
            'Color', colors(r, :), 'LineStyle', styles{r}, 'DisplayName', lbl);
    end
    xlabel('Time (s)'); ylabel('Torque (N\cdotm)');
    title(joint_names{j}, 'FontWeight', 'bold');
    if j == 1, legend('Location', 'northeast', 'FontSize', 7); end
end
sgtitle('5-DOF Joint Actuator Torques vs Time across Payload Variations', ...
    'FontSize', 12, 'FontWeight', 'bold');

saveas(fig2, fullfile(outDir, 'joint_torques_vs_time.fig'));
exportgraphics(fig2, fullfile(outDir, 'joint_torques_vs_time.png'), 'Resolution', 300);
close(fig2);
fprintf('Saved joint_torques_vs_time.png (300 DPI)\n');

% FIGURE 3: RMS Error Comparison Bar Chart
fig3 = figure('Name', 'RMS Error Comparison', 'Position', [100, 100, 1100, 600], ...
              'Color', 'w', 'Visible', 'off');

% Data matrix: 6 rows (runs) x 5 cols (joints)
rms_matrix = zeros(length(results), 5);
group_labels = cell(length(results), 1);
for r = 1:length(results)
    rms_matrix(r, :) = results(r).RMS;
    group_labels{r} = sprintf('%s (%.1f kg)', results(r).Controller, results(r).Payload_kg);
end

% Transpose for grouped bar: 5 joints on x-axis, 6 bars per group
b = bar(rms_matrix', 'grouped');
grid on; box on;
set(gca, 'XTickLabel', {'J1 (Yaw)', 'J2 (Shoulder)', 'J3 (Elbow)', 'J4 (WristP)', 'J5 (WristR)'});
ylabel('RMS Tracking Error (rad)');
title('RMS Tracking Error Comparison: CTC vs PID Across Payload Variations', ...
    'FontSize', 11, 'FontWeight', 'bold');
legend(group_labels, 'Location', 'northwest', 'FontSize', 8);

for k = 1:length(b)
    b(k).FaceColor = colors(k, :);
end

saveas(fig3, fullfile(outDir, 'rms_error_comparison_barchart.fig'));
exportgraphics(fig3, fullfile(outDir, 'rms_error_comparison_barchart.png'), 'Resolution', 300);
close(fig3);
fprintf('Saved rms_error_comparison_barchart.png (300 DPI)\n');

close_system(model_nominal, 0);
close_system(model_ctc, 0);
fprintf('\nAll benchmarking tasks completed successfully!\n');
