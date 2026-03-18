clc
clear; close all;

% from main_proj2_maps...
load('trajectory_data.mat');

waypoints_all = [trajectory.x', -trajectory.y']; % allignment frame system

% robot parameter
max_v = 0.22;
max_omega = 2.84;
lookahead = 2;  % L
goal_tol = 0.008;
dt = 0.1;
k_rho = 1.1;
alpha_tol = pi/4;

% number of nodes for each path
n_1 = trajectory.n(1);
n_2 = trajectory.n(2);
n_3 = trajectory.n(3);
n_4 = trajectory.n(4);
n_wp_all = size(waypoints_all,1);

% connection to ROS2
node = ros2node("/matlab_tb3");
odomSub = ros2subscriber(node, "/odom", "nav_msgs/Odometry");
[velPub, velMsg] = ros2publisher(node, "/cmd_vel", "geometry_msgs/TwistStamped");
rate = ros2rate(node, 10); % freq rate 10 Hz
pause(3);

odomMsg = receive(odomSub, 0.1);
xR = odomMsg.pose.pose.position.x;
yR = odomMsg.pose.pose.position.y;
% choose closer waypoint from where start (pitagora)
d0 = hypot(waypoints_all(:,1)-xR, waypoints_all(:,2)-yR);
[~, i_wp] = min(d0);

%% first trajectory
waypoints = waypoints_all(1:n_1,:);
idx_target_old = lookahead;
n_curr = n_1;

while i_wp <= n_curr
    odomMsg = receive(odomSub, 0.5);
    xR = odomMsg.pose.pose.position.x;
    yR = odomMsg.pose.pose.position.y;
    q = [odomMsg.pose.pose.orientation.w, odomMsg.pose.pose.orientation.x, ...
        odomMsg.pose.pose.orientation.y, odomMsg.pose.pose.orientation.z];
    eul = quat2eul(q, 'ZYX');
    thetaR = eul(1);

    d0 = hypot(waypoints(i_wp:end,1)-xR, waypoints(i_wp:end,2)-yR);
    [~, i_wp_rel] = min(d0);
    % avoid come back (d0 done with remaining points, so relatiev index)
    i_wp = i_wp_rel + i_wp - 1;
    i_wp = min(i_wp, n_curr);

    % target index
    idx_target = min(i_wp + lookahead, n_curr);
    idx_target = max(idx_target, idx_target_old);
    idx_target_old = idx_target;

    % coordinates terget
    xT = waypoints(idx_target,1);
    yT = waypoints(idx_target,2);
    rho = hypot(xT-xR, yT-yR);

    % control direction and velocity
    if idx_target < n_curr || rho > goal_tol
        target_theta = atan2(yT-yR, xT-xR);
        alpha = wrapToPi(target_theta - thetaR);

        L = lookahead * 0.05;   
        v = min(k_rho*rho, 0.3);  % proportional controller
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
n_curr = n_2 - n_1 + 1;

odomMsg = receive(odomSub, 0.5);
xR = odomMsg.pose.pose.position.x;
yR = odomMsg.pose.pose.position.y;
d0 = hypot(waypoints(:,1) - xR, waypoints(:,2) - yR);  
[~, i_wp] = min(d0);  
idx_target_old = lookahead;

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

    idx_target = min(i_wp + lookahead, n_curr);
    idx_target = max(idx_target, idx_target_old);
    idx_target_old = idx_target;

    xT = waypoints(idx_target,1);
    yT = waypoints(idx_target,2);
    rho = hypot(xT-xR, yT-yR);

    if idx_target < n_curr || rho > goal_tol
        target_theta = atan2(yT-yR, xT-xR);
        alpha = wrapToPi(target_theta - thetaR);

        L = lookahead * 0.05;   
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

%% third trajectory
waypoints = waypoints_all(n_2+1:n_3,:);
i_wp = 1;
idx_target_old = lookahead;
n_curr = n_3 - n_2;

while i_wp <= n_curr

    odomMsg = receive(odomSub, 0.5);
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

    idx_target = min(i_wp + lookahead, n_curr);
    idx_target = max(idx_target, idx_target_old);
    idx_target_old = idx_target;

    xT = waypoints(idx_target,1);
    yT = waypoints(idx_target,2);
    rho = hypot(xT-xR, yT-yR);

    if idx_target < n_curr || rho > goal_tol
        target_theta = atan2(yT-yR, xT-xR);
        alpha = wrapToPi(target_theta - thetaR);

        L = lookahead * 0.05;   
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
i_wp = 1;
idx_target_old = lookahead;
n_curr = n_4 - n_3;

while i_wp <= n_curr

    odomMsg = receive(odomSub, 0.5);
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


    idx_target = min(i_wp + lookahead, n_curr);
    idx_target = max(idx_target, idx_target_old);
    idx_target_old = idx_target;

    xT = waypoints(idx_target,1);
    yT = waypoints(idx_target,2);
    rho = hypot(xT-xR, yT-yR);

    if idx_target < n_curr || rho > goal_tol
        target_theta = atan2(yT-yR, xT-xR);
        alpha = wrapToPi(target_theta - thetaR);

        L = lookahead * 0.05;   
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