clear
clc
close all

% plotta palle in gazebo, accende nodi e fa ruotare

%%
worldName = "default";

modelPath = fullfile(pwd, "red_sphere.sdf");
x = 0;
y = +1;
cmd = sprintf('unset LD_LIBRARY_PATH; source /opt/ros/jazzy/setup.bash; ros2 run ros_gz_sim create -world %s -file %s -name ball_red -x %.2f -y %.2f -z 0.2', ...
    worldName, modelPath, x, y);
[status,out] = system(['bash -c "' cmd '"']);
if status ~= 0
    disp("Spawn failed:");
    disp(out);
else
    disp("Spawn successful:");
    disp(out);
end

modelPath = fullfile(pwd, "big_red_sphere.sdf");
x = -3;
y = 0;
cmd = sprintf('unset LD_LIBRARY_PATH; source /opt/ros/jazzy/setup.bash; ros2 run ros_gz_sim create -world %s -file %s -name ball_big_red -x %.2f -y %.2f -z 0.4', ...
    worldName, modelPath, x, y);
[status,out] = system(['bash -c "' cmd '"']);
if status ~= 0
    disp("Spawn failed:");
    disp(out);
else
    disp("Spawn successful:");
    disp(out);
end


modelPath = fullfile(pwd, "big_purple_sphere.sdf");
x = 3;
y = 0;
cmd = sprintf('unset LD_LIBRARY_PATH; source /opt/ros/jazzy/setup.bash; ros2 run ros_gz_sim create -world %s -file %s -name ball_purple -x %.2f -y %.2f -z 0.4', ...
    worldName, modelPath, x, y);
[status,out] = system(['bash -c "' cmd '"']);
if status ~= 0
    disp("Spawn failed:");
    disp(out);
else
    disp("Spawn successful:");
    disp(out);
end


%% Subscriber
node = ros2node("/matlab_tb3");
odomSub = ros2subscriber(node,"/odom","nav_msgs/Odometry"); % POSITION
LidarSub = ros2subscriber(node,"/scan","sensor_msgs/LaserScan"); % DISTANCE
imgSub = ros2subscriber(node,"/camera/image_raw","sensor_msgs/Image"); % CAMERA

omega = 0.2;
