clc
clear; close all;

load('trajectory_data.mat');

waypoints_all = [trajectory.x', -trajectory.y'];

% robot parameter
max_v = 0.22;
max_omega = 2.84;
lookahead_pts_near = 2;
lookahead_pts_far = 8;
goal_tol = 0.008;
dt = 0.1;
k_rho_near = 0.8;
k_rho_far = 1.1;

n_1 = trajectory.n(1);
n_2 = trajectory.n(2);
n_3 = trajectory.n(3);
n_4 = trajectory.n(4);

ang_thresh = 5*pi/180;  % threeshold to consider a segment straight
min_run = 5; % minimum number of straight consecutive segments 
n_wp_all = size(waypoints_all,1);

% angle of each segment
theta_seg = zeros(n_wp_all-1,1);
for i = 1:n_wp_all-1
    dx = waypoints_all(i+1,1) - waypoints_all(i,1);
    dy = waypoints_all(i+1,2) - waypoints_all(i,2);
    theta_seg(i) = atan2(dy,dx);
end

is_straight_all = false(n_wp_all,1); % flag if point is in a straight segment
run_len_fwd_all = ones(n_wp_all,1); % counter how many segments are straight

run_len_fwd_all(1) = 53; 
% if low angle variation, consider straigth
dth_vect = [];
run = 1;
for i = n_wp_all-1:-1:2
    dth = wrapToPi(theta_seg(i) - theta_seg(i-1));
    dth_vect = [dth_vect dth];
    if abs(dth) < ang_thresh
        run = run + 1;
    else
        run = 1;
    end
    run_len_fwd_all(i)=run;
end
% if more straigth segments-->consider straigth
for i = 1:n_wp_all
    if run_len_fwd_all(i) >= min_run
        is_straight_all(i) = true;
    end
end

% creation nodes 
node = ros2node("/matlab_tb3");
odomSub = ros2subscriber(node, "/odom", "nav_msgs/Odometry");
[velPub, velMsg] = ros2publisher(node, "/cmd_vel", "geometry_msgs/TwistStamped");
rate = ros2rate(node, 10);
pause(3);

n_wp = n_wp_all;

odomMsg = receive(odomSub, 0.1);
xR = odomMsg.pose.pose.position.x;
yR = odomMsg.pose.pose.position.y;
d0 = hypot(waypoints_all(:,1)-xR, waypoints_all(:,2)-yR);
[~, i_wp] = min(d0);
disp(['Start da WP #', num2str(i_wp), ...
    ' (', num2str(waypoints_all(i_wp,1),2), ', ', num2str(waypoints_all(i_wp,2),2), ')']);

log_x = []; log_y = []; v_profile = []; omega_profile = []; v_odom = []; omega_odom = []; log_theta = [];
wp_tol = 0.008;
alpha_tol = pi/4;

%% first trajectory
waypoints = waypoints_all(1:n_1,:);
is_straight = is_straight_all(1:n_1);
run_len_fwd = run_len_fwd_all(1:n_1);
idx_target_old = lookahead_pts_near;

% control
while i_wp <= n_1

    odomMsg = receive(odomSub, 0.2);
    xR = odomMsg.pose.pose.position.x;
    yR = odomMsg.pose.pose.position.y;
    q = [odomMsg.pose.pose.orientation.w, odomMsg.pose.pose.orientation.x, ...
        odomMsg.pose.pose.orientation.y, odomMsg.pose.pose.orientation.z];
    eul = quat2eul(q, 'ZYX');
    thetaR = eul(1);

    % waypoint piu vicini
    d0 = hypot(waypoints_all(i_wp:end,1)-xR, waypoints_all(i_wp:end,2)-yR);
    [~, i_wp_rel] = min(d0);
    i_wp = i_wp_rel + i_wp - 1;
    i_wp = min(i_wp, n_1)

    % adaptive look-ahead
    if is_straight(i_wp) && run_len_fwd(i_wp) > (lookahead_pts_far + 3)
        idx_target = min(i_wp + lookahead_pts_far, n_1);
        k_rho = k_rho_far;
    else
        idx_target = min(i_wp + lookahead_pts_near, n_1);
        k_rho = k_rho_near;
    end
    idx_target = max(idx_target, idx_target_old);
    idx_target_old = idx_target;

    xT = waypoints(idx_target,1);
    yT = waypoints(idx_target,2);
    rho = hypot(xT-xR, yT-yR);

    % control (same before)
    if idx_target < n_1 || rho > goal_tol
        target_theta = atan2(yT-yR, xT-xR);
        alpha = wrapToPi(target_theta - thetaR);

        L = lookahead_pts_near * 0.05;   
        v = min(k_rho*rho, 0.3);
        v = max(-max_v, min(max_v, v));
        alpha_eff = max(-pi/2, min(pi/2, alpha));
        omega = (2 * v / L) * sin(alpha_eff);
        omega = max(-max_omega, min(max_omega, omega));

        if abs(alpha) > alpha_tol
            v = 0.0;
        end
    else
        break;
    end

    velMsg.twist.linear.x  = v;
    velMsg.twist.angular.z = omega;
    velMsg.header.stamp    = ros2time(node, 'now');
    send(velPub, velMsg);
    waitfor(rate);
end

velMsg.twist.linear.x = 0;
velMsg.twist.angular.z = 0;
velMsg.header.stamp = ros2time(node, 'now');
send(velPub, velMsg);
pause(3);

%% second trajectory
waypoints = waypoints_all(n_1:n_2,:);
is_straight = is_straight_all(n_1:n_2);
run_len_fwd = run_len_fwd_all(n_1:n_2);
n_curr = n_2 - n_1 + 1;

odomMsg = receive(odomSub, 0.2);
xR = odomMsg.pose.pose.position.x;
yR = odomMsg.pose.pose.position.y;
d0 = hypot(waypoints(:,1) - xR, waypoints(:,2) - yR);  
[~, i_wp] = min(d0);  

idx_target_old = lookahead_pts_near;

while i_wp <= n_curr

    odomMsg = receive(odomSub, 0.2);
    xR = odomMsg.pose.pose.position.x;
    yR = odomMsg.pose.pose.position.y;
    q = [odomMsg.pose.pose.orientation.w, odomMsg.pose.pose.orientation.x, ...
        odomMsg.pose.pose.orientation.y, odomMsg.pose.pose.orientation.z];
    eul = quat2eul(q, 'ZYX');
    thetaR = eul(1);

    d0 = hypot(waypoints(i_wp:end,1)-xR, waypoints(i_wp:end,2)-yR);
    [~, i_wp_rel] = min(d0);
    i_wp = i_wp_rel + i_wp - 1;
    i_wp = min(i_wp, n_curr)

    if is_straight(i_wp) && run_len_fwd(i_wp) > (lookahead_pts_far + 3)
        idx_target = min(i_wp + lookahead_pts_far, n_curr);
        k_rho = k_rho_far;
    else
        idx_target = min(i_wp + lookahead_pts_near, n_curr);
        k_rho = k_rho_near;
    end
    idx_target = max(idx_target, idx_target_old);
    idx_target_old = idx_target;

    xT = waypoints(idx_target,1);
    yT = waypoints(idx_target,2);
    rho = hypot(xT-xR, yT-yR);

    if idx_target < n_curr || rho > goal_tol
        target_theta = atan2(yT-yR, xT-xR);
        alpha = wrapToPi(target_theta - thetaR);

        L = lookahead_pts_near * 0.05;   
        v = min(k_rho*rho, 0.3);
        v = max(-max_v, min(max_v, v));
        alpha_eff = max(-pi/2, min(pi/2, alpha));
        omega = (2 * v / L) * sin(alpha_eff);
        omega = max(-max_omega, min(max_omega, omega));

        if abs(alpha) > alpha_tol
            v = 0.0;
        end
    else
        break;
    end

    velMsg.twist.linear.x  = v;
    velMsg.twist.angular.z = omega;
    velMsg.header.stamp    = ros2time(node, 'now');
    send(velPub, velMsg);

    log_x(end+1) = xR;  log_y(end+1) = yR; log_theta(end+1) = thetaR;
    v_profile(end+1) = v;  omega_profile(end+1) = omega;
    v_odom (end+1) = odomMsg.twist.twist.linear.x;
    omega_odom(end + 1) = odomMsg.twist.twist.angular.z;

    waitfor(rate);
end

velMsg.twist.linear.x = 0;
velMsg.twist.angular.z = 0;
velMsg.header.stamp = ros2time(node, 'now');
send(velPub, velMsg);
pause(3);

%% third trajectory
waypoints = waypoints_all(n_2+1:n_3,:);
is_straight = is_straight_all(n_2+1:n_3);
run_len_fwd = run_len_fwd_all(n_2+1:n_3);
i_wp = 1;
idx_target_old = lookahead_pts_near;
n_curr = n_3 - n_2;

while i_wp <= n_curr

    odomMsg = receive(odomSub, 0.7);
    xR = odomMsg.pose.pose.position.x;
    yR = odomMsg.pose.pose.position.y;
    q = [odomMsg.pose.pose.orientation.w, odomMsg.pose.pose.orientation.x, ...
        odomMsg.pose.pose.orientation.y, odomMsg.pose.pose.orientation.z];
    eul = quat2eul(q, 'ZYX');
    thetaR = eul(1);

    d0 = hypot(waypoints(i_wp:end,1)-xR, waypoints(i_wp:end,2)-yR);
    [~, i_wp_rel] = min(d0);
    i_wp = i_wp_rel + i_wp - 1;
    i_wp = min(i_wp, n_curr)


    if is_straight(i_wp) && run_len_fwd(i_wp) > (lookahead_pts_far + 3)
        idx_target = min(i_wp + lookahead_pts_far, n_curr);
        k_rho = k_rho_far;
    else
        idx_target = min(i_wp + lookahead_pts_near, n_curr);
        k_rho = k_rho_near;
    end
    idx_target = max(idx_target, idx_target_old);
    idx_target_old = idx_target;

    xT = waypoints(idx_target,1);
    yT = waypoints(idx_target,2);
    rho = hypot(xT-xR, yT-yR);

    if idx_target < n_curr || rho > goal_tol
        target_theta = atan2(yT-yR, xT-xR);
        alpha = wrapToPi(target_theta - thetaR);

        L = lookahead_pts_near * 0.05;   
        v = min(k_rho*rho, 0.3);
        v = max(-max_v, min(max_v, v));
        alpha_eff = max(-pi/2, min(pi/2, alpha));
        omega = (2 * v / L) * sin(alpha_eff);
        omega = max(-max_omega, min(max_omega, omega));

        if abs(alpha) > alpha_tol
            v = 0.0;
        end
    else
        break;
    end

    velMsg.twist.linear.x  = v;
    velMsg.twist.angular.z = omega;
    velMsg.header.stamp    = ros2time(node, 'now');
    send(velPub, velMsg);

    waitfor(rate);
end

velMsg.twist.linear.x = 0;
velMsg.twist.angular.z = 0;
velMsg.header.stamp = ros2time(node, 'now');
send(velPub, velMsg);
pause(3);

%% fourth trajectory
waypoints = waypoints_all(n_3+1:n_4,:);
is_straight = is_straight_all(n_3+1:n_4);
run_len_fwd = run_len_fwd_all(n_3+1:n_4);
i_wp = 1;
idx_target_old = lookahead_pts_near;
n_curr = n_4 - n_3;

while i_wp <= n_curr

    odomMsg = receive(odomSub, 0.7);
    xR = odomMsg.pose.pose.position.x;
    yR = odomMsg.pose.pose.position.y;
    q = [odomMsg.pose.pose.orientation.w, odomMsg.pose.pose.orientation.x, ...
        odomMsg.pose.pose.orientation.y, odomMsg.pose.pose.orientation.z];
    eul = quat2eul(q, 'ZYX');
    thetaR = eul(1);

    d0 = hypot(waypoints(i_wp:end,1)-xR, waypoints(i_wp:end,2)-yR);
    [~, i_wp_rel] = min(d0);
    i_wp = i_wp_rel + i_wp - 1;
    i_wp = min(i_wp, n_curr)


    if is_straight(i_wp) && run_len_fwd(i_wp) > (lookahead_pts_far + 3)
        idx_target = min(i_wp + lookahead_pts_far, n_curr);
        k_rho = k_rho_far;
    else
        idx_target = min(i_wp + lookahead_pts_near, n_curr);
        k_rho = k_rho_near;
    end
    idx_target = max(idx_target, idx_target_old);
    idx_target_old = idx_target;

    xT = waypoints(idx_target,1);
    yT = waypoints(idx_target,2);
    rho = hypot(xT-xR, yT-yR);

    if idx_target < n_curr || rho > goal_tol
        target_theta = atan2(yT-yR, xT-xR);
        alpha = wrapToPi(target_theta - thetaR);

        L = lookahead_pts_near * 0.05;   
        v = min(k_rho*rho, 0.3);
        v = max(-max_v, min(max_v, v));
        alpha_eff = max(-pi/2, min(pi/2, alpha));
        omega = (2 * v / L) * sin(alpha_eff);
        omega = max(-max_omega, min(max_omega, omega));

        if abs(alpha) > alpha_tol
            v = 0.0;
        end
    else
        break;
    end

    velMsg.twist.linear.x  = v;
    velMsg.twist.angular.z = omega;
    velMsg.header.stamp    = ros2time(node, 'now');
    send(velPub, velMsg);

    waitfor(rate);
end

velMsg.twist.linear.x = 0;
velMsg.twist.angular.z = 0;
velMsg.header.stamp = ros2time(node, 'now');
send(velPub, velMsg);
pause(3);