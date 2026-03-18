clear
clc
close all

%% look-ahead = 8
load('trajectory_data.mat')
waypoints = [trajectory.x', -trajectory.y']; 

bagFolder = "replay_8LA"; 
bag = ros2bagreader(bagFolder);

sel = select(bag, "Topic", "/odom");
t0 = sel.StartTime; t1 = sel.EndTime;
sel = select(sel, "Time", [t0 t1]);
odomMsgs = readMessages(sel);

% initial data or final not important
n_in = 150; 
n_end = 200;
N = length(odomMsgs)-(n_in+n_end); %n_in and n_out are added to cut the dead parts of the recording
log_x = zeros(N,1);
log_y = zeros(N,1);
log_theta = zeros(N,1);
time_vect = zeros(N,1);
v_odom = zeros(N,1);
omega_odom = zeros(N,1);
j = 0;

% extract data from bag
for i=n_in:N+n_in
    j=j+1;
    log_x(j) = odomMsgs{i}.pose.pose.position.x;
    log_y(j) = odomMsgs{i}.pose.pose.position.y;
    time_vect(j) = double(odomMsgs{i}.header.stamp.sec) + double(odomMsgs{i}.header.stamp.nanosec).*1e-9;
    v_odom(j) = odomMsgs{i}.twist.twist.linear.x;
    omega_odom(j) = odomMsgs{i}.twist.twist.angular.z;
    quater = odomMsgs{i}.pose.pose.orientation;
    q = quaternion([quater.w quater.x quater.y quater.z]);
    eul = quat2eul(q);
    log_theta(j) = eul(1);
end
% time normalization
time_vect = time_vect-time_vect(1);

figure
plot(waypoints(:,1), waypoints(:,2), 'b-', 'LineWidth',2); hold on;
title('Ideal Trajectory'); xlabel('X [m]'); ylabel('Y [m]')
xlim([-4.5 9])
axis equal; grid on;
print(gcf,'ideal_trajectory.eps', '-depsc', '-r300');

figure; 
plot(waypoints(:,1), waypoints(:,2), 'b-', 'LineWidth',2); hold on;
plot(log_x, log_y, 'r-', 'LineWidth',1.5);
plot(log_x(1), log_y(1), 'yo', 'MarkerSize',12, 'MarkerFaceColor','y');
plot(log_x(end), log_y(end), 'go', 'MarkerSize',12, 'MarkerFaceColor','g');
title('Trajectory'); xlabel('X [m]'); ylabel('Y [m]')
legend('Ideal','Real','Initial Point', 'Final Point');
xlim([-4.5 9])
axis equal; grid on;
print(gcf,'trajectory.eps', '-depsc', '-r300');


figure
subplot(2,1,1)
plot(time_vect, v_odom, 'LineWidth',1.5);
grid on
title('Velocity Magnitude')
xlabel('t [s]');
ylabel('v [m/s]');
xlim([0 time_vect(end)*1.01])

subplot(2,1,2)
plot(time_vect, omega_odom, 'LineWidth',1.5);
grid on
title('Angular Velocity')
xlabel('t [s]');
ylabel('\omega [rad/s]');
hold on
xlim([0 time_vect(end)*1.01])
print(gcf,'velocity.eps', '-depsc', '-r300');

figure
subplot(3,1,1)
plot(time_vect, log_x, 'LineWidth',2)
grid on
title('X Coordinate')
xlabel('t [s]');
ylabel('x [m]');
hold on
xlim([0 time_vect(end)*1.01])

subplot(3,1,2)
plot(time_vect, log_y,'LineWidth',2)
grid on
title('Y Coordinate')
xlabel('t [s]');
ylabel('y [m]');
hold on
xlim([0 time_vect(end)*1.01])

subplot(3,1,3)
plot(time_vect, log_theta,'LineWidth',2)
grid on
title('Orientation')
xlabel('t [s]');
ylabel('\theta [rad]');
hold on
xlim([0 time_vect(end)*1.01])

print(gcf,'position.eps', '-depsc', '-r300');

pos = [log_x, log_y];
n_pos = size(pos, 1);
err_pos = zeros(n_pos, 1);

% error position (RMS)
for i = 1:n_pos
    dist = sqrt((waypoints(:,1) - pos(i,1)).^2 + (waypoints(:,2) - pos(i,2)).^2);    
    err_pos(i) = min(dist);
end

rmse = sqrt(mean(err_pos.^2))
time_vect(end)

%% look-ahead = 2
clear, close all
load('trajectory_data.mat')
waypoints = [trajectory.x', -trajectory.y']; 

bagFolder = "replay_2LA"; 
bag = ros2bagreader(bagFolder);

sel = select(bag, "Topic", "/odom");
t0 = sel.StartTime; t1 = sel.EndTime;
sel = select(sel, "Time", [t0 t1]);
odomMsgs = readMessages(sel);

n_in = 100;
n_end = 275;
N = length(odomMsgs)-(n_in+n_end);
log_x = zeros(N,1);
log_y = zeros(N,1);
log_theta = zeros(N,1);
time_vect = zeros(N,1);
v_odom = zeros(N,1);
omega_odom = zeros(N,1);
j = 0;

for i=n_in:N+n_in
    j=j+1;
    log_x(j) = odomMsgs{i}.pose.pose.position.x;
    log_y(j) = odomMsgs{i}.pose.pose.position.y;
    time_vect(j) = double(odomMsgs{i}.header.stamp.sec) + double(odomMsgs{i}.header.stamp.nanosec).*1e-9;
    v_odom(j) = odomMsgs{i}.twist.twist.linear.x;
    omega_odom(j) = odomMsgs{i}.twist.twist.angular.z;
    quater = odomMsgs{i}.pose.pose.orientation;
    q = quaternion([quater.w quater.x quater.y quater.z]);
    eul = quat2eul(q);
    log_theta(j) = eul(1);
end
time_vect = time_vect-time_vect(1);


figure; 
plot(waypoints(:,1), waypoints(:,2), 'b-', 'LineWidth',2); hold on;
plot(log_x, log_y, 'r-', 'LineWidth',1.5);
plot(log_x(1), log_y(1), 'yo', 'MarkerSize',12, 'MarkerFaceColor','y');
plot(log_x(end), log_y(end), 'go', 'MarkerSize',12, 'MarkerFaceColor','g');
title('Trajectory'); xlabel('X [m]'); ylabel('Y [m]')
legend('Ideal','Real','Initial Point', 'Final Point');
xlim([-4.5 9])
axis equal; grid on;
print(gcf,'trajectory_noisy.eps', '-depsc', '-r300');


figure
subplot(2,1,1)
plot(time_vect, v_odom, 'LineWidth',1.5);
grid on
title('Velocity Magnitude')
xlabel('t [s]');
ylabel('v [m/s]');
hold on
xlim([0 time_vect(end)*1.01])

subplot(2,1,2)
plot(time_vect, omega_odom, 'LineWidth',1.5);
grid on
title('Angular Velocity')
xlabel('t [s]');
ylabel('\omega [rad/s]');
hold on
xlim([0 time_vect(end)*1.01])
print(gcf,'velocity_noisy.eps', '-depsc', '-r300');

figure
subplot(3,1,1)
plot(time_vect, log_x, 'LineWidth',2)
grid on
title('X Coordinate')
xlabel('t [s]');
ylabel('x [m]');
hold on
xlim([0 time_vect(end)*1.01])

subplot(3,1,2)
plot(time_vect, log_y,'LineWidth',2)
grid on
title('Y Coordinate')
xlabel('t [s]');
ylabel('y [m]');
hold on
xlim([0 time_vect(end)*1.01])

subplot(3,1,3)
plot(time_vect, log_theta,'LineWidth',2)
grid on
title('Orientation')
xlabel('t [s]');
ylabel('\theta [rad]');
hold on
xlim([0 time_vect(end)*1.01])

print(gcf,'position_noisy.eps', '-depsc', '-r300');

pos = [log_x, log_y];
n_pos = size(pos, 1);
err_pos = zeros(n_pos, 1);

for i = 1:n_pos
    dist = sqrt((waypoints(:,1) - pos(i,1)).^2 + (waypoints(:,2) - pos(i,2)).^2);    
    err_pos(i) = min(dist);
end

rmse = sqrt(mean(err_pos.^2))
time_vect(end)