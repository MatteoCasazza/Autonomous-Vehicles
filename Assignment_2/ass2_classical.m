clear
clc

% target x,y,theta
waypoints= [0 0 0 ;
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

%robot and controller parameters
max_v = 0.2;      % [m/s]
max_omega = 0.4;  % [rad/s]
% compromise between stability and performance:
% too high-->spring effect=overshoot,persistent oscillations around target position
% too low-->too much long time
k_dist = 0.6;      % [1/s]
k_ang = 1.4;    % [1/s]
% si da priorita a orientamento, prima si orienta e poi va verso target,
% per evitare zig zag, overshoot o slittamenti

% tolerances lower to have higher performance
tol = 0.02;  % [m]
tol_ang= 0.01;  % [rad]
dt = 0.1;         % [s]

% ROS2 comunications
node = ros2node("/matlab_tb3");
odomSub = ros2subscriber(node, "/odom", "nav_msgs/Odometry");
[velPub, velMsg] = ros2publisher(node, "/cmd_vel", "geometry_msgs/TwistStamped");
% attendi per 1 secondo per dare tempo ai topic di inizializzarsi
pause(1); 

n_wp = size(waypoints,1);
i_wp = 2; 

% loop per waypoints
while i_wp <= n_wp
    t_loop = tic; 

    % lettura odometry and orientation
    odomMsg = receive(odomSub, 2.0);
    xR = odomMsg.pose.pose.position.x;
    yR = odomMsg.pose.pose.position.y;
    q = odomMsg.pose.pose.orientation;
    qv = [q.w q.x q.y q.z];
    eul = quat2eul(qv, 'ZYX');
    thetaR = eul(1);
    
    %error position
    xT = waypoints(i_wp,1);
    yT = waypoints(i_wp,2);
    dx = xT - xR;
    dy = yT - yR;
    dist = sqrt(dx^2 + dy^2);
    
    % when far away
    if dist>=tol
        target_theta = atan2(dy, dx);
        ang = wrapToPi(target_theta - thetaR);
        % P controller
        v = k_dist * dist;
        omega = k_ang * ang;
        % saturation
        v = min(max_v, max(-max_v, v));
        omega = min(max_omega, max(-max_omega, omega));
       % for stability: avoid robot goes forward looking backward
       % (prima ruota e poi avanza)
       % se errore angolare è troppo grande
        if abs(ang) > pi/2
            v = 0.0;
        end

        velMsg.twist.linear.x = v;
        velMsg.twist.angular.z = omega;
        % pubblicazione comando
        t_now = datetime('now', 'TimeZone', 'UTC');
        s = posixtime(t_now);
        velMsg.header.stamp.sec = int32(floor(s));
        velMsg.header.stamp.nanosec = uint32(round((s - floor(s))*1e9));
        send(velPub, velMsg);
    
    % vicino a waypoint
    elseif dist < tol
        target_theta = waypoints(i_wp,3);
        ang = wrapToPi(target_theta - thetaR);
        % controller make robot only rotate for the right orientation
        v = 0.0;
        omega = k_ang * ang;
        v = min(max_v, max(-max_v, v));
        omega = min(max_omega, max(-max_omega, omega));


        velMsg.twist.linear.x = v;
        velMsg.twist.angular.z = omega;
        t_now = datetime('now', 'TimeZone', 'UTC');
        s = posixtime(t_now);
        velMsg.header.stamp.sec = int32(floor(s));
        velMsg.header.stamp.nanosec = uint32(round((s - floor(s))*1e9));
        send(velPub, velMsg);

        % obiettivo raggiunt0 --> passa a new waypoint
        if abs(ang)<tol_ang
            i_wp = i_wp + 1
            pause(1.5)
        end
    end
    
    % controllo del tempo per garantire freq=10Hz
    elapsed = toc(t_loop);
    pause(max(0, dt - elapsed)); % dt = 0.1

end

% arresto finale
velMsg.twist.linear.x = 0.0;
velMsg.twist.angular.z = 0.0;
t_now = datetime('now', 'TimeZone', 'UTC');
s = posixtime(t_now);
velMsg.header.stamp.sec = int32(floor(s));
velMsg.header.stamp.nanosec = uint32(round((s - floor(s))*1e9));
send(velPub, velMsg);
pause(2)
