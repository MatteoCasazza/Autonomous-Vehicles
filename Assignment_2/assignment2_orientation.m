clear
clc
close all

% RESULTS: longer travel time and distance, but more continous position
% profiles, more activity in lateral acceleration as robots steers more 

waypoints = [0 0 0 ;
    5 0 pi/2;
    5 5 5/4*pi;
    -5 -5 pi/2;
    -5 5 0;
    0 0 0;
    3 3 3/4*pi;
    -3 0 3/2*pi;
    0 -3 pi/4;
    3 0 pi/2;
    0 0 3/2*pi];

max_v = 0.2;      
max_omega = 0.4;  
% larger tolerances since more difficult than before
tol = 0.05;  
tol_ang = 0.03; 
dt = 0.1;         
% finale
k_dist = 0.3; 
k_ang = 0.8; 
k_beta = -0.15; 

node = ros2node("/matlab_tb3");
odomSub = ros2subscriber(node, "/odom", "nav_msgs/Odometry");
[velPub, velMsg] = ros2publisher(node, "/cmd_vel", "geometry_msgs/TwistStamped");
pause(1.5);

n_wp = size(waypoints, 1);
i_wp = 2; 

while i_wp <= n_wp
    t_loop = tic;

    odomMsg = receive(odomSub, 2.0);
    xR = odomMsg.pose.pose.position.x;
    yR = odomMsg.pose.pose.position.y;
    q = odomMsg.pose.pose.orientation;
    eul = quat2eul([q.w q.x q.y q.z], 'ZYX');
    thetaR = eul(1); 

    xT = waypoints(i_wp, 1);
    yT = waypoints(i_wp, 2);
    thetaT = waypoints(i_wp, 3);
    dx = xT - xR;
    dy = yT - yR;
    dist = sqrt(dx^2 + dy^2); 
    target_theta = atan2(dy, dx);
    ang = wrapToPi(target_theta - thetaR);
    % error waypoint orienation
    beta = wrapToPi(thetaT - target_theta);

    if dist >= tol
        v = k_dist * dist;
        % combination of error towards waypoint and final orientation
        omega = k_ang * ang + k_beta * beta;
        if abs(ang) > pi/2
            v = 0.0;
        end
    else
        v = 0.0;
        error_theta = wrapToPi(thetaT - thetaR);
        omega = k_ang * error_theta; 
        
        if abs(error_theta) < tol_ang
            i_wp = i_wp + 1;
            pause(1.5); % Piccola pausa per stabilità
        end
    end
    
    %saturation
    v = min(max_v, max(-max_v, v));
    omega = min(max_omega, max(-max_omega, omega));
    %send command velocities to node
    velMsg.twist.linear.x = v;
    velMsg.twist.angular.z = omega;
    t_now = datetime('now', 'TimeZone', 'UTC');
    s = posixtime(t_now);
    velMsg.header.stamp.sec = int32(floor(s));
    velMsg.header.stamp.nanosec = uint32(round((s - floor(s))*1e9));
    send(velPub, velMsg);

    % ciclo a fre quenza fissa 10 Hz
    elapsed = toc(t_loop);
    pause(max(0, dt - elapsed));
end

velMsg.twist.linear.x = 0.0;
velMsg.twist.angular.z = 0.0;
t = datetime('now', 'TimeZone', 'UTC');
s = posixtime(t);
velMsg.header.stamp.sec = int32(floor(s));
velMsg.header.stamp.nanosec = uint32(round((s - floor(s))*1e9));
send(velPub, velMsg);
