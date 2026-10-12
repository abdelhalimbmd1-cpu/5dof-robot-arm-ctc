function tau = robot_controller_fcn(u)
% ROBOT_CONTROLLER_FCN 5-DOF Joint Controller
% Modes:
%   ctrl_mode = 1: Decoupled Joint PID (tuned for nominal payload = 0)
%   ctrl_mode = 2: Computed Torque Control (CTC, feedback linearization + PD)
%
% Input vector u (27x1):
%   u(1:5)   = q(1:5)       Measured joint positions (rad)
%   u(6:10)  = qd(1:5)      Measured joint velocities (rad/s)
%   u(11:15) = q_d(1:5)     Desired joint positions (rad)
%   u(16:20) = qd_d(1:5)    Desired joint velocities (rad/s)
%   u(21:25) = qdd_d(1:5)   Desired joint accelerations (rad/s^2)
%   u(26)    = ctrl_mode    1 = PID, 2 = CTC
%   u(27)    = t            Simulation time (s)

global robot_nominal Kp_pid Kd_pid Ki_pid Kp_ctc Kd_ctc prev_t e_int;

u = u(:);

q     = u(1:5);
qd    = u(6:10);
q_d   = u(11:15);
qd_d  = u(16:20);
qdd_d = u(21:25);
ctrl_mode = round(u(26));
t     = u(27);

% Default gains if not set in base workspace
if isempty(Kp_pid)
    try Kp_pid = evalin('base', 'Kp_pid'); catch
        Kp_pid = [350; 600; 450; 250; 80];
    end
end
if isempty(Kd_pid)
    try Kd_pid = evalin('base', 'Kd_pid'); catch
        Kd_pid = [45; 75; 55; 30; 10];
    end
end
if isempty(Ki_pid)
    try Ki_pid = evalin('base', 'Ki_pid'); catch
        Ki_pid = [25; 45; 35; 20; 5];
    end
end
if isempty(Kp_ctc)
    try Kp_ctc = evalin('base', 'Kp_ctc'); catch
        Kp_ctc = [225; 225; 225; 225; 225];
    end
end
if isempty(Kd_ctc)
    try Kd_ctc = evalin('base', 'Kd_ctc'); catch
        Kd_ctc = [30; 30; 30; 30; 30];
    end
end

% Integral error management with monotonic dt check and anti-windup clamping
if isempty(prev_t) || t < prev_t
    prev_t = t;
    e_int = zeros(5, 1);
elseif t > prev_t
    dt = t - prev_t;
    if dt > 0 && dt < 0.05
        e_int = e_int + (q_d - q) * dt;
        e_int = max(-5.0, min(5.0, e_int)); % Anti-windup
    end
    prev_t = t;
end

e  = q_d - q;
ed = qd_d - qd;

if ctrl_mode == 1
    % Controller A: Decoupled Joint PID
    kp = Kp_pid(:);
    kd = Kd_pid(:);
    ki = Ki_pid(:);
    tau = kp .* e + kd .* ed + ki .* e_int;
else
    % Controller B: Computed Torque Control (CTC)
    % tau = M(q)*(qdd_d + Kd*(qd_d - qd) + Kp*(q_d - q)) + C(q, qd)*qd + G(q)
    kp_ctc = Kp_ctc(:);
    kd_ctc = Kd_ctc(:);
    qdd_star = qdd_d + kd_ctc .* ed + kp_ctc .* e;
    
    if isempty(robot_nominal)
        nomModel = 'FiveDOF_Robot_Arm';
        if ~bdIsLoaded(nomModel)
            load_system(nomModel);
        end
        robot_nominal = importrobot(nomModel);
        robot_nominal.DataFormat = 'column';
        robot_nominal.Gravity = [0; 0; -9.81];
    end
    
    % Inverse dynamics evaluates M(q)*qdd* + C(q, qd)*qd + G(q)
    tau = inverseDynamics(robot_nominal, q, qd, qdd_star);
end

% Joint torque saturation limits (N*m)
tau_limit = [500; 800; 500; 300; 100];
tau = max(-tau_limit, min(tau_limit, tau));

tau = reshape(tau, 5, 1);
end
