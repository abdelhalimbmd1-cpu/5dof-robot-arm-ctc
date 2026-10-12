function traj = trajectory_generator_fcn(t)
% TRAJECTORY_GENERATOR_FCN
% Generates a smooth, continuous C^2 quintic polynomial Pick-and-Place trajectory
% for the 5-DOF Robot Arm across a 9-phase industrial mission.
%
% Output: traj = [q_d(1:5); qd_d(1:5); qdd_d(1:5)] (15x1 vector)

% 5-DOF Joint Waypoints (rad): [q1(Yaw); q2(Shoulder); q3(Elbow); q4(WristP); q5(WristR)]
w_home    = [0.0; 0.0; 0.0; 0.0; 0.0];
w_A_hover = deg2rad([-35.0;  25.0; -35.0; 10.0; 0.0]);
w_A_pick  = deg2rad([-35.0;  50.0; -60.0; 10.0; 0.0]);
w_B_hover = deg2rad([ 35.0;  25.0; -35.0; 10.0; 0.0]);
w_B_place = deg2rad([ 35.0;  50.0; -60.0; 10.0; 0.0]);

times = [0.0, 2.0, 3.5, 4.5, 6.0, 8.5, 10.0, 11.0, 12.0, 14.0];
waypoints = [
    w_home, ...       % 1: t=0.0s  (Home Standby)
    w_A_hover, ...    % 2: t=2.0s  (Phase 1: Approach Pick Station A)
    w_A_pick, ...     % 3: t=3.5s  (Phase 2: Descend onto object at Point A)
    w_A_pick, ...     % 4: t=4.5s  (Phase 3: Grasp dwell at Point A)
    w_A_hover, ...    % 5: t=6.0s  (Phase 4: Lift object to clearance height)
    w_B_hover, ...    % 6: t=8.5s  (Phase 5: Transfer swing to Station B)
    w_B_place, ...    % 7: t=10.0s (Phase 6: Descend onto fixture at Point B)
    w_B_place, ...    % 8: t=11.0s (Phase 7: Release dwell at Point B)
    w_B_hover, ...    % 9: t=12.0s (Phase 8: Retract / Lift from Point B)
    w_home            % 10: t=14.0s (Phase 9: Return to Home Standby)
];

if t <= times(1)
    q_d = waypoints(:, 1);
    qd_d = zeros(5, 1);
    qdd_d = zeros(5, 1);
elseif t >= times(end)
    q_d = waypoints(:, end);
    qd_d = zeros(5, 1);
    qdd_d = zeros(5, 1);
else
    % Find active interval
    k = 1;
    for i = 1:(length(times) - 1)
        if t >= times(i) && t < times(i+1)
            k = i;
            break;
        end
    end
    
    t0 = times(k);
    t1 = times(k+1);
    dt = t1 - t0;
    
    tau = (t - t0) / dt;
    tau = max(0.0, min(1.0, tau));
    
    % Quintic polynomial blending (zero initial & final velocity/accel)
    s = 10*tau^3 - 15*tau^4 + 6*tau^5;
    sd = (30*tau^2 - 60*tau^3 + 30*tau^4) / dt;
    sdd = (60*tau - 180*tau^2 + 120*tau^3) / (dt^2);
    
    delta_w = waypoints(:, k+1) - waypoints(:, k);
    
    q_d = waypoints(:, k) + s * delta_w;
    qd_d = sd * delta_w;
    qdd_d = sdd * delta_w;
end

traj = [q_d; qd_d; qdd_d];
end
