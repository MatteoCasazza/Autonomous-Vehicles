clear
clc
close all
% CONTROLLER STANLEY:
% SUITED FOR DISCRETE WAYPOINTS, CONSTANT VELOCITY, PRECISE FINISH
% IT FOLLOWS THE SEGMENT BETWEEN 2 POINTS (NOT THE NEXT POINT)

% RESULTS: less travel time and distance, rotates also when starts, high
% peak in longitudinal velocity since a strong braking force is needed when
% close to the target, more steering --> more angular velocity
% best time performance

%% WAYPOINTS
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

%% PARAMETERS
% choose max possible speed as reference
%--> to minimize travel time 
max_v = 0.2;                
max_omega = 0.4;            
v_ref = 0.2; 
% defines how fast the Burger reacts when it is not on the right path
k_e = 0.8;                 
tol = 0.03;           
tol_ang = 0.02;         
dt = 0.1;                   
L = 0.16; % wheelbase of burger robot

%% ALGORITHM
node = ros2node("/matlab_tb3_stanley");
odomSub = ros2subscriber(node, "/odom", "nav_msgs/Odometry");
[velPub, velMsg] = ros2publisher(node, "/cmd_vel", "geometry_msgs/TwistStamped");

n_wp = size(waypoints, 1);
i_wp = 2;  

while i_wp <= n_wp
    t_loop = tic;  

    odomMsg = receive(odomSub, 2.0);
    xR = odomMsg.pose.pose.position.x;
    yR = odomMsg.pose.pose.position.y;
    q = odomMsg.pose.pose.orientation;
    qv = [q.w q.x q.y q.z];
    eul = quat2eul(qv, 'ZYX');
    thetaR = eul(1);  

    pause(dt)

    xT = waypoints(i_wp, 1);
    yT = waypoints(i_wp, 2);
    theta_T = waypoints(i_wp, 3);
    
    dx = xT - xR;
    dy = yT - yR;
    dist = sqrt(dx^2 + dy^2);

    if dist > tol
        target_theta = atan2(dy, dx);
        ang = wrapToPi(target_theta - thetaR);

        xS = waypoints(i_wp-1, 1); % S: starting point
        yS = waypoints(i_wp-1, 2); 
        dx_seg = xT - xS;
        dy_seg = yT - yS;
        dist_seg = sqrt(dx_seg^2 + dy_seg^2);
        
        % lateral error= perpendicular distance from straight line
        if dist_seg > 1e-6
            e_lateral = ((yT - yS) * xR - (xT - xS) * yR + xT * yS - yT * xS) / dist_seg;
        else
            e_lateral = sqrt((xR-xT)^2 + (yR-yT)^2);
        end

        if abs(v_ref) < 1e-6
            delta = ang;
        else
            % control law:
            % ang=allineament
            % atan(...)= lateral correction
            % delta=virtual steering
            delta = ang + atan2(k_e * e_lateral, v_ref);
        end
        
        omega_ref = (v_ref/L)*tan(delta);
        v_cmd = v_ref;
        
        % modulation velocity based on orientation
        % more disallignement-->more slow down to improve stability, reduce
        % overshoot
        if abs(ang) > pi/3
            v_cmd = 0.05;      
        elseif abs(ang) > pi/6
            v_cmd = 0.1;       
        end
        
        % saturation
        v_cmd = min(max_v, max(-max_v, v_cmd));
        omega_cmd = min(max_omega, max(-max_omega, omega_ref));
    
    % quando arrivi vicino a waypoint, si usa P controller per aggiustare
    % orientamento
    elseif dist <= tol
        ang = wrapToPi(theta_T - thetaR);
        v_cmd = 0.0;
        
        if abs(ang) > tol_ang
            omega_cmd = k_e * ang;
            omega_cmd = min(max_omega, max(-max_omega, omega_cmd));
        else
            omega_cmd = 0.0;
        end        
    end
    
    velMsg.twist.linear.x = v_cmd;
    velMsg.twist.angular.z = omega_cmd;
    
    t_now = datetime('now', 'TimeZone', 'UTC');
    s = posixtime(t_now);
    velMsg.header.stamp.sec = int32(floor(s));
    velMsg.header.stamp.nanosec = uint32(round((s - floor(s)) * 1e9));
    send(velPub, velMsg);
    
    % cambio waypoint
    if dist <= tol && abs(wrapToPi(waypoints(i_wp, 3) - thetaR)) < tol_ang
        if abs(wrapToPi(waypoints(i_wp, 3) - thetaR)) < tol_ang
            i_wp = i_wp + 1;
            pause(1.5);  
        end
    end
    
    elapsed = toc(t_loop);
    pause(max(0, dt - elapsed));
end

velMsg.twist.linear.x = 0.0;
velMsg.twist.angular.z = 0.0;
t_now = datetime('now', 'TimeZone', 'UTC');
s = posixtime(t_now);
velMsg.header.stamp.sec = int32(floor(s));
velMsg.header.stamp.nanosec = uint32(round((s - floor(s)) * 1e9));
send(velPub, velMsg);
