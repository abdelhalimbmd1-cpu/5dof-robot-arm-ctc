%% build_5dof_ctc_model.m
% Bulletproof builder for 5-DOF Robot Arm Simscape Multibody model
% Configures torque actuation, sensing, payload parameterization,
% and Interpreted MATLAB Function controllers for CTC and PID.

targetDir = 'C:\Users\ZORO\Downloads\5dof-robot-manipulator-1.snapshot.2';
sourceModel = 'C:\Users\ZORO\robot_arm_5dof\Clean_Arm_Verification.slx';
cd(targetDir);

fprintf('========================================================================\n');
fprintf('       BUILDING 5-DOF ROBOT ARM SIMSCAPE CTC SIMULATION MODEL           \n');
fprintf('========================================================================\n\n');

%% Step 1: Create FiveDOF_Robot_Arm.slx (Nominal CAD Model for importrobot)
fprintf('1. Creating nominal model FiveDOF_Robot_Arm.slx ...\n');
nomModel = 'FiveDOF_Robot_Arm';
if bdIsLoaded(nomModel), close_system(nomModel, 0); end
copyfile(sourceModel, [nomModel '.slx']);
load_system(nomModel);
set_param(nomModel, 'SimMechanicsOpenEditorOnUpdate', 'off');
set_param(nomModel, 'EnablePacing', 'off');

% Update STEP file paths to current target directory
stepFiles = {
    'Base_Solid', 'Base.STEP';
    'Solid_1',    'Link C22.STEP';
    'Solid_2',    'Link C33 Connector.STEP';
    'Solid_3',    'Link C33.STEP';
    'Solid_4',    'Tool Holder.STEP';
    'Solid_5',    'Revolver.STEP';
    'Solid_Combiner', 'Combiner.STEP';
    'Solid_Cutter',   'Cutter.STEP'
};

for i = 1:size(stepFiles, 1)
    blk = [nomModel '/' stepFiles{i, 1}];
    fullStep = fullfile(targetDir, stepFiles{i, 2});
    fullStep = strrep(fullStep, '\', '/');
    if ~isempty(find_system(nomModel, 'SearchDepth', 1, 'Name', stepFiles{i, 1}))
        set_param(blk, 'ExtGeomFileName', fullStep);
    end
end

% Set Angle_1..5 Constant values to 0 instead of q_test
for i = 1:5
    angBlk = [nomModel sprintf('/Angle_%d', i)];
    if ~isempty(find_system(nomModel, 'SearchDepth', 1, 'Name', sprintf('Angle_%d', i)))
        set_param(angBlk, 'Value', '0');
    end
end

save_system(nomModel);
close_system(nomModel);
fprintf('   FiveDOF_Robot_Arm.slx saved successfully.\n\n');

%% Step 2: Create FiveDOF_Robot_Arm_CTC.slx (Actuated Plant)
fprintf('2. Creating actuated model FiveDOF_Robot_Arm_CTC.slx ...\n');
ctcModel = 'FiveDOF_Robot_Arm_CTC';
if bdIsLoaded(ctcModel), close_system(ctcModel, 0); end
copyfile([nomModel '.slx'], [ctcModel '.slx']);
load_system(ctcModel);
set_param(ctcModel, 'SimMechanicsOpenEditorOnUpdate', 'off');
set_param(ctcModel, 'EnablePacing', 'off');

% Remove open-loop Angle and S2P blocks from Clean_Arm_Verification
for i = 1:5
    angBlk = [ctcModel sprintf('/Angle_%d', i)];
    if ~isempty(find_system(ctcModel, 'SearchDepth', 1, 'Name', sprintf('Angle_%d', i)))
        delete_block(angBlk);
    end
    s2pOld = [ctcModel sprintf('/S2P_%d', i)];
    if ~isempty(find_system(ctcModel, 'SearchDepth', 1, 'Name', sprintf('S2P_%d', i)))
        delete_block(s2pOld);
    end
end

% Configure 5 Revolute Joints for Torque Actuation and Sensing
joints = {'Joint1_Yaw', 'Joint2_Shoulder', 'Joint3_Elbow', 'Joint4_WristP', 'Joint5_WristR'};
for i = 1:5
    jPath = [ctcModel '/' joints{i}];
    set_param(jPath, 'TorqueActuationMode', 'InputTorque');
    set_param(jPath, 'MotionActuationMode', 'ComputedMotion');
    set_param(jPath, 'SensePosition', 'on');
    set_param(jPath, 'SenseVelocity', 'on');
end

% Add Parameterized Payload Solid attached to Cutter
fprintf('3. Adding parameterized Payload solid...\n');
payloadTrans = [ctcModel '/Payload_Transform'];
if isempty(find_system(ctcModel, 'SearchDepth', 1, 'Name', 'Payload_Transform'))
    add_block('sm_lib/Frames and Transforms/Rigid Transform', payloadTrans, 'Position', [1500, 320, 1540, 360]);
    set_param(payloadTrans, 'TranslationMethod', 'Cartesian');
    set_param(payloadTrans, 'TranslationCartesianOffset', '[0 0 0.17]');
    set_param(payloadTrans, 'TranslationCartesianOffsetUnits', 'm');
end

payloadSolid = [ctcModel '/Payload_Solid'];
if isempty(find_system(ctcModel, 'SearchDepth', 1, 'Name', 'Payload_Solid'))
    add_block('sm_lib/Body Elements/Brick Solid', payloadSolid, 'Position', [1580, 320, 1620, 360]);
    set_param(payloadSolid, 'BrickDimensions', '[0.05 0.05 0.05]');
    set_param(payloadSolid, 'InertiaType', 'Custom');
    set_param(payloadSolid, 'Mass', 'payload_mass');
    set_param(payloadSolid, 'MomentsOfInertia', '[1e-4 1e-4 1e-4]');
    set_param(payloadSolid, 'ProductsOfInertia', '[0 0 0]');
    set_param(payloadSolid, 'GraphicDiffuseColor', '[0.9 0.2 0.2]');
end

try
    add_line(ctcModel, 'Joint5_WristR/RConn1', 'Payload_Transform/LConn1', 'autorouting', 'smart');
    add_line(ctcModel, 'Payload_Transform/RConn1', 'Payload_Solid/RConn1', 'autorouting', 'smart');
catch
end

% Add Actuation (S2P) and Sensing (P2S) Converters
fprintf('4. Adding Simulink-PS and PS-Simulink converters...\n');
for i = 1:5
    s2pName = [ctcModel sprintf('/S2P_Tau%d', i)];
    if isempty(find_system(ctcModel, 'SearchDepth', 1, 'Name', sprintf('S2P_Tau%d', i)))
        add_block('nesl_utility/Simulink-PS Converter', s2pName, 'Position', [200 + i*150, 100, 230 + i*150, 120]);
        set_param(s2pName, 'Unit', 'N*m');
    end
    
    p2sQName = [ctcModel sprintf('/P2S_Q%d', i)];
    if isempty(find_system(ctcModel, 'SearchDepth', 1, 'Name', sprintf('P2S_Q%d', i)))
        add_block('nesl_utility/PS-Simulink Converter', p2sQName, 'Position', [200 + i*150, 600, 230 + i*150, 620]);
        set_param(p2sQName, 'Unit', 'rad');
    end
    
    p2sWName = [ctcModel sprintf('/P2S_W%d', i)];
    if isempty(find_system(ctcModel, 'SearchDepth', 1, 'Name', sprintf('P2S_W%d', i)))
        add_block('nesl_utility/PS-Simulink Converter', p2sWName, 'Position', [200 + i*150, 650, 230 + i*150, 670]);
        set_param(p2sWName, 'Unit', 'rad/s');
    end
end

% Connect physical signals to joints
for i = 1:5
    jName = joints{i};
    jPorts = get_param([ctcModel '/' jName], 'PortHandles');
    
    s2pPorts = get_param([ctcModel sprintf('/S2P_Tau%d', i)], 'PortHandles');
    try add_line(ctcModel, s2pPorts.RConn(1), jPorts.LConn(2), 'autorouting', 'smart'); catch; end
    
    p2sQPorts = get_param([ctcModel sprintf('/P2S_Q%d', i)], 'PortHandles');
    try add_line(ctcModel, jPorts.RConn(2), p2sQPorts.LConn(1), 'autorouting', 'smart'); catch; end
    
    p2sWPorts = get_param([ctcModel sprintf('/P2S_W%d', i)], 'PortHandles');
    try add_line(ctcModel, jPorts.RConn(3), p2sWPorts.LConn(1), 'autorouting', 'smart'); catch; end
end

% Add Mux_Q and Mux_W (width 5)
fprintf('5. Adding signal routing and multiplexers...\n');
if isempty(find_system(ctcModel, 'SearchDepth', 1, 'Name', 'Mux_Q'))
    add_block('simulink/Signal Routing/Mux', [ctcModel '/Mux_Q'], 'Position', [1050, 580, 1055, 640]);
    set_param([ctcModel '/Mux_Q'], 'Inputs', '5');
end
if isempty(find_system(ctcModel, 'SearchDepth', 1, 'Name', 'Mux_W'))
    add_block('simulink/Signal Routing/Mux', [ctcModel '/Mux_W'], 'Position', [1050, 660, 1055, 720]);
    set_param([ctcModel '/Mux_W'], 'Inputs', '5');
end

for i = 1:5
    try add_line(ctcModel, sprintf('P2S_Q%d/1', i), sprintf('Mux_Q/%d', i), 'autorouting', 'smart'); catch; end
    try add_line(ctcModel, sprintf('P2S_W%d/1', i), sprintf('Mux_W/%d', i), 'autorouting', 'smart'); catch; end
end

% Add Demux_Tau (5 outputs)
if isempty(find_system(ctcModel, 'SearchDepth', 1, 'Name', 'Demux_Tau'))
    add_block('simulink/Signal Routing/Demux', [ctcModel '/Demux_Tau'], 'Position', [300, 40, 305, 160]);
    set_param([ctcModel '/Demux_Tau'], 'Outputs', '5');
end
for i = 1:5
    try add_line(ctcModel, sprintf('Demux_Tau/%d', i), sprintf('S2P_Tau%d/1', i), 'autorouting', 'smart'); catch; end
end

% Add Clock
if isempty(find_system(ctcModel, 'SearchDepth', 1, 'Name', 'Clock'))
    add_block('simulink/Sources/Clock', [ctcModel '/Clock'], 'Position', [50, 750, 80, 770]);
end

% Add Trajectory Generator (Interpreted MATLAB Function)
trajBlock = [ctcModel '/Trajectory_Generator'];
if isempty(find_system(ctcModel, 'SearchDepth', 1, 'Name', 'Trajectory_Generator'))
    add_block('simulink/User-Defined Functions/Interpreted MATLAB Function', trajBlock, 'Position', [120, 740, 200, 780]);
    set_param(trajBlock, 'MATLABFcn', 'trajectory_generator_fcn');
end
try add_line(ctcModel, 'Clock/1', 'Trajectory_Generator/1', 'autorouting', 'smart'); catch; end

% Add Constant for ctrl_mode
if isempty(find_system(ctcModel, 'SearchDepth', 1, 'Name', 'ctrl_mode'))
    add_block('simulink/Sources/Constant', [ctcModel '/ctrl_mode'], 'Position', [50, 810, 80, 830]);
    set_param([ctcModel '/ctrl_mode'], 'Value', 'ctrl_mode');
end

% Add Mux for Controller Input: [q (5), qd (5), traj (15), ctrl_mode (1), Clock (1)] -> 5 inputs, total width 27
muxCtrl = [ctcModel '/Mux_CtrlIn'];
if isempty(find_system(ctcModel, 'SearchDepth', 1, 'Name', 'Mux_CtrlIn'))
    add_block('simulink/Signal Routing/Mux', muxCtrl, 'Position', [1120, 700, 1125, 860]);
    set_param(muxCtrl, 'Inputs', '5');
end
try
    add_line(ctcModel, 'Mux_Q/1', 'Mux_CtrlIn/1', 'autorouting', 'smart');
    add_line(ctcModel, 'Mux_W/1', 'Mux_CtrlIn/2', 'autorouting', 'smart');
    add_line(ctcModel, 'Trajectory_Generator/1', 'Mux_CtrlIn/3', 'autorouting', 'smart');
    add_line(ctcModel, 'ctrl_mode/1', 'Mux_CtrlIn/4', 'autorouting', 'smart');
    add_line(ctcModel, 'Clock/1', 'Mux_CtrlIn/5', 'autorouting', 'smart');
catch
end

% Add Controller block (Interpreted MATLAB Function)
ctrlBlock = [ctcModel '/Robot_Controller'];
if isempty(find_system(ctcModel, 'SearchDepth', 1, 'Name', 'Robot_Controller'))
    add_block('simulink/User-Defined Functions/Interpreted MATLAB Function', ctrlBlock, 'Position', [1160, 760, 1260, 800]);
    set_param(ctrlBlock, 'MATLABFcn', 'robot_controller_fcn');
end
try
    add_line(ctcModel, 'Mux_CtrlIn/1', 'Robot_Controller/1', 'autorouting', 'smart');
    add_line(ctcModel, 'Robot_Controller/1', 'Demux_Tau/1', 'autorouting', 'smart');
catch
end

% Add To Workspace blocks for logging
logs = {
    'Mux_Q', 'q_out';
    'Mux_W', 'qd_out';
    'Trajectory_Generator', 'traj_ref';
    'Robot_Controller', 'tau_out';
};
for i = 1:size(logs, 1)
    toWsName = [ctcModel '/ToWs_' logs{i, 2}];
    if isempty(find_system(ctcModel, 'SearchDepth', 1, 'Name', ['ToWs_' logs{i, 2}]))
        add_block('simulink/Sinks/To Workspace', toWsName, 'Position', [1320, 600 + i*60, 1380, 630 + i*60]);
        set_param(toWsName, 'VariableName', logs{i, 2});
        set_param(toWsName, 'SaveFormat', 'Timeseries');
        set_param(toWsName, 'SampleTime', '0.005');
    end
end
try
    add_line(ctcModel, 'Mux_Q/1', 'ToWs_q_out/1', 'autorouting', 'smart');
    add_line(ctcModel, 'Mux_W/1', 'ToWs_qd_out/1', 'autorouting', 'smart');
    add_line(ctcModel, 'Trajectory_Generator/1', 'ToWs_traj_ref/1', 'autorouting', 'smart');
    add_line(ctcModel, 'Robot_Controller/1', 'ToWs_tau_out/1', 'autorouting', 'smart');
catch
end

% Configure Solver
set_param(ctcModel, 'StopTime', '14.0');
set_param(ctcModel, 'Solver', 'ode15s');
set_param(ctcModel, 'MaxStep', '0.005');
set_param(ctcModel, 'RelTol', '1e-3');
set_param(ctcModel, 'AbsTol', '1e-4');

save_system(ctcModel);
close_system(ctcModel);
fprintf('   FiveDOF_Robot_Arm_CTC.slx built and saved successfully!\n');
fprintf('========================================================================\n');
exit;
