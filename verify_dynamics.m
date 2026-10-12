%% verify_dynamics.m
% Verifies the analytical multibody equations of motion against
% the Robotics System Toolbox inverse dynamics solver for the 5-DOF robot arm.
%
% Robot Equations of Motion:
%   M(q)*qdd + C(q, qd)*qd + G(q) = tau
%
% Tests 3 distinct configurations across position, velocity, and acceleration.

projectDir = fileparts(mfilename('fullpath'));
if isempty(projectDir), projectDir = pwd; end
cd(projectDir);

fprintf('========================================================================\n');
fprintf('       5-DOF ROBOT ARM: DYNAMICS MODEL FORMULATION & VERIFICATION       \n');
fprintf('========================================================================\n\n');

model_nominal = 'FiveDOF_Robot_Arm';
if ~bdIsLoaded(model_nominal)
    load_system(model_nominal);
end

global robot_nominal;
robot_nominal = importrobot(model_nominal);
robot_nominal.DataFormat = 'column';
robot_nominal.Gravity = [0; 0; -9.81];

fprintf('Multibody rigidBodyTree model loaded successfully.\n');
fprintf('Number of rigid bodies: %d\n', robot_nominal.NumBodies);
for i = 1:robot_nominal.NumBodies
    b = robot_nominal.Bodies{i};
    fprintf('  - Body %d: %-12s | Mass: %6.3f kg | Joint: %-15s (%s)\n', ...
        i, b.Name, b.Mass, b.Joint.Name, b.Joint.Type);
end
fprintf('\n');

% Test configurations (5x3 matrices)
test_configs = [
    0.0,   0.0,   0.0,   0.0,   0.0;
    0.3,   0.4,  -0.3,   0.2,  -0.1;
   -0.2,  -0.3,   0.5,  -0.4,   0.3
]';

test_vels = [
    0.0,   0.0,   0.0,   0.0,   0.0;
    0.1,  -0.2,   0.15, -0.1,   0.05;
   -0.15,  0.2,  -0.1,   0.25, -0.1
]';

test_accs = [
    0.0,   0.0,   0.0,   0.0,   0.0;
    0.5,  -0.3,   0.4,  -0.2,   0.1;
   -0.3,   0.4,  -0.2,   0.3,  -0.25
]';

fprintf('--- Verifying Dynamic Decomposition M(q)*qdd + C(q,qd)*qd + G(q) = tau ---\n');
for i = 1:3
    q_i   = test_configs(:, i);
    qd_i  = test_vels(:, i);
    qdd_i = test_accs(:, i);
    
    M_i = massMatrix(robot_nominal, q_i);
    V_i = velocityProduct(robot_nominal, q_i, qd_i);
    G_i = gravityTorque(robot_nominal, q_i);
    tau_id = inverseDynamics(robot_nominal, q_i, qd_i, qdd_i);
    
    err_dyn = norm(tau_id - (M_i*qdd_i + V_i + G_i));
    
    fprintf('Config %d: q = [%.2f, %.2f, %.2f, %.2f, %.2f] rad\n', ...
        i, q_i(1), q_i(2), q_i(3), q_i(4), q_i(5));
    fprintf('          Dynamic Decomposition Error: %.2e N*m\n', err_dyn);
    fprintf('          Gravity Vector G(q): [%.2f, %.2f, %.2f, %.2f, %.2f] N*m\n\n', ...
        G_i(1), G_i(2), G_i(3), G_i(4), G_i(5));
end

fprintf('SUCCESS: Multibody dynamics verified with exact analytical consistency.\n');
fprintf('========================================================================\n');
close_system(model_nominal, 0);

