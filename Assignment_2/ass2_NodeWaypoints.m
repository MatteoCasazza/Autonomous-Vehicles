clear
clc
close all

% MATLAB Controller
%    |
%    |  /wp_request  (Int32)
%    ↓
% Waypoint Server (Python)
%    |
%    |  /next_waypoint (PoseStamped)
%    ↓
% MATLAB Controller

% respect to original type there is a delay

max_v = 0.2;     
max_omega = 0.4; 
k_dist = 0.6;     
k_ang = 1.4;   
tol = 0.02; 
tol_ang = 0.01;
dt = 0.1;        

node = ros2node("/matlab_tb3");
% publisher vcomand velocity
[velPub, velMsg] = ros2publisher(node, "/cmd_vel", "geometry_msgs/TwistStamped");
% publisher richiesta waypoint
[wpReqPub, wpReqMsg] = ros2publisher(node, "/wp_request", "std_msgs/Int32");
% subscriber waypoint
wpSub = ros2subscriber(node, "/next_waypoint", "geometry_msgs/PoseStamped");
% subscriber odometry
odomSub = ros2subscriber(node, "/odom", "nav_msgs/Odometry");

pause(1); 

k_wp = 1;              
last_wp = false;

% gestisce un waypoint alla volta
while ~last_wp
    % richiesta waypoint a python
    wpReqMsg.data = int32(k_wp);
    send(wpReqPub, wpReqMsg);
    wpMsg = receive(wpSub);      
    xT = wpMsg.pose.position.x;
    yT = wpMsg.pose.position.y;
    qT = wpMsg.pose.orientation;
    thetaT = atan2(2*qT.w*qT.z, 1 - 2*qT.z*qT.z);  % from quaternions to radiants

    goal   = false;    % posizione raggiunta
    goal_angle = false;   % orientamento raggiunto

    % controllo movimento per arrivare a goal
    % P controller
    while ~(goal && goal_angle)
        t_loop = tic;

        odomMsg = receive(odomSub, 2.0);
        xR = odomMsg.pose.pose.position.x;
        yR = odomMsg.pose.pose.position.y;
        q  = odomMsg.pose.pose.orientation;
        qv = [q.w q.x q.y q.z];
        eul = quat2eul(qv, 'ZYX');
        thetaR = eul(1);

        dx  = xT - xR;
        dy  = yT - yR;
        dist = hypot(dx, dy);

        if ~goal && dist >= tol
            target_theta = atan2(dy, dx);
            ang = wrapToPi(target_theta - thetaR);

            v = k_dist * dist;
            omega = k_ang * ang;
            v = min(max_v, max(-max_v, v));
            omega = min(max_omega, max(-max_omega, omega));

            if abs(ang) > pi/2
                v = 0.0;
            end

            velMsg.twist.linear.x  = v;
            velMsg.twist.angular.z = omega;

            if dist < tol
                goal = true;
            end

        else
            goal = true;
            ang = wrapToPi(thetaT - thetaR);   % differenza heading

            v = 0.0;
            omega = k_ang * ang;
            omega = min(max_omega, max(-max_omega, omega));

            velMsg.twist.linear.x  = v;
            velMsg.twist.angular.z = omega;

            if abs(ang) < tol_ang
                goal_angle = true;
            end
        end

        t_now = datetime('now', 'TimeZone', 'UTC');
        s = posixtime(t_now);
        velMsg.header.stamp.sec     = int32(floor(s));
        velMsg.header.stamp.nanosec = uint32(round((s - floor(s))*1e9));
        send(velPub, velMsg);
        t_elapsed = toc(t_loop);

        pause(max(0, dt - t_elapsed));
    end

    k_wp = k_wp + 1;
    if k_wp > 11          
        last_wp = true;
    end

    pause(1.5); 
end


velMsg.twist.linear.x = 0.0;
velMsg.twist.angular.z = 0.0;
t = datetime('now', 'TimeZone', 'UTC');
s = posixtime(t);
velMsg.header.stamp.sec = int32(floor(s));
velMsg.header.stamp.nanosec = uint32(round((s - floor(s))*1e9));
send(velPub, velMsg);