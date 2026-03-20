clear
close all
clc

% extract data pos
load('pos_data.mat')
pos.time_vect = pos.time_vect(1:end-1);
e = 1e-5;

%% plot
figure(1);
plot(pos.X, pos.Y, Linewidth= 3, Color= 'k');
grid on;
title('Robot position from /odom topic');
xlabel('X [m]');
ylabel('Y [m]');
hold on

%% speed
vx = diff([0; pos.X])./diff([0; pos.time_vect]);
vy = diff([0; pos.Y])./diff([0; pos.time_vect]);
vy(abs(vy)<e) = 0;
vx(abs(vx)<e) = 0;

v_tot = sqrt(vx.^2 + vy.^2);
v_tot(abs(v_tot)<e) = 0;

% theta = unwrap(pos.theta);

theta=(atan2(vy,vx));
theta(isnan(theta))=0;
figure
plot(pos.time_vect, theta, 'LineWidth',2)
hold on
plot(pos.time_vect, pos.theta, 'LineWidth',2)
grid on
xlabel('t [s]')
ylabel('\theta [rad]')
title('Angular position over time')

% Plot speed magnitude
figure('Name','Speed Magnitude');
plot(pos.time_vect, v_tot, 'LineWidth', 2, 'Color', 'b');
grid on;
title('Speed Magnitude over Time');
xlabel('Time [s]');
ylabel('Speed [m/s]');
theta=pos.theta;
print(gcf,'speed_magnitude.eps', '-depsc', '-r300');

figure
plot(pos.time_vect, theta, 'LineWidth',2)
grid on
xlabel('t [s]')
ylabel('\theta [rad]')
title('Angular position over time')



dtheta = diff([0; theta])./diff([0;pos.time_vect]);
% dtheta = diff(theta)./diff(pos.time_vect(1:end));

%Se vedo un salto assurdo in un campione, lo considero un errore numerico e
% lo sostituisco con il valore medio dei due vicini
for ii = 1:length(dtheta)-1
    if abs(dtheta(ii)-dtheta(ii+1)) > 2
        dtheta(ii+1) = 0.5*(dtheta(ii)+dtheta(ii+2));
    end
end

dtheta(abs(dtheta)<e) = 0;

% dtheta plot
figure('Name','Change in Angle over Time');
plot(pos.time_vect, dtheta, 'LineWidth', 2, 'Color', 'b');
grid on;
title('Angular Speed');
xlabel('Time [s]');
ylabel('\omega [rad/s]');
hold on
print(gcf,'angular_speed.eps', '-depsc', '-r300');

figure(1)
plot(pos.X(dtheta>e),pos.Y(dtheta>e),'og', MarkerSize=4,MarkerFaceColor='g')
hold on
plot(pos.X(dtheta<-e),pos.Y(dtheta<-e),'ob', MarkerSize=4,MarkerFaceColor='b')
hold on
plot(pos.X(abs(dtheta)<e),pos.Y(abs(dtheta)<e),'or', MarkerSize=4,MarkerFaceColor='r')
hold on

% dydx = diff(pos.Y)./diff(pos.X);
% constantSlopeIndices = zeros(length(dydx),1);
% for ii = 1:length(dydx)-1
%     if abs(dydx(ii) - dydx(ii+1)) < e
%         % Store the index of constant slope
%         constantSlopeIndices(ii) = ii;
%     end
% end
% 
% constantSlopeIndices = constantSlopeIndices(constantSlopeIndices~=0);
% figure(1)
% plot(pos.X(constantSlopeIndices(1:end-1)),pos.Y(constantSlopeIndices(1:end-1)),'or', MarkerSize=4,MarkerFaceColor='r')
legend('trajectory','left','right', 'straight')
print(gcf,'colors_direction.eps', '-depsc', '-r300');

%%
figure
scatter(pos.X, pos.Y, 50, v_tot, 'filled');
colorbar;
title('Velocity Colormap');
xlabel('X [m]');
ylabel('Y [m]');
colormap('jet')
grid on
print(gcf,'speed_colormap.eps', '-depsc', '-r300');
% clim ([-5 5])

%% interpolation
time_10hz = pos.time_vect(1:5:end);
v_int = interp1(pos.time_vect, v_tot, time_10hz);

figure('Name','Velocity compare')
plot(time_10hz, v_int, LineWidth=2, Color='r')
grid on
hold on
plot(pos.time_vect, v_tot, 'LineWidth', 2, 'Color', 'b')
legend('interpolated','not interpolated')

dtheta_int = interp1(pos.time_vect, dtheta, time_10hz);

figure('Name','Angle compare')
plot(time_10hz, dtheta_int, LineWidth=2, Color='r')
grid on
hold on
plot(pos.time_vect, dtheta, 'LineWidth', 2, 'Color', 'b')
legend('interpolated','not interpolated')

vel.v_tot = v_int;
vel.dtheta = dtheta_int;
vel.time = time_10hz;

save('velocity.mat',"vel");
%%
close all

% Prepare velocity messages for each time instant
node= ros2node("/matlab_tb3"); % new node in the ID Domain
[velPub, velMsg] = ros2publisher(node,"/cmd_vel","geometry_msgs/TwistStamped");

dt = 0.1; % intervallo di invio (s)
last = tic;

for i = 1:length(v_int)
    t_loop = tic;

    velMsg.twist.linear.x = v_int(i);
    velMsg.twist.angular.z = dtheta_int(i);
    i/length(v_tot)*100
    
    % Timestamp "now"
    t = datetime('now','TimeZone','UTC');
    s = posixtime(t);
    velMsg.header.stamp.sec = int32(floor(s));
    velMsg.header.stamp.nanosec = uint32(round((s - floor(s))*1e9));
    
    % Invio messaggio
    send(velPub, velMsg);
    
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
