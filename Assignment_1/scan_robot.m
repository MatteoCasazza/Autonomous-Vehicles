clear
clc
close all

load('scanMsgs.mat');
load('scantf.mat');
load("scantf_static.mat");
load('pos.mat')

% pos_robot=pos;

%%
time_vect=zeros(length(scanMsgs),1);
N=length(scanMsgs);

for i=1:length(scanMsgs)
    time_vect(i)=double(scanMsgs{i}.header.stamp.sec)+double(scanMsgs{i}.header.stamp.nanosec)*1e-9;
end

pos.X=pos.X(3:2:end);
pos.Y=pos.Y(3:2:end);
pos.Z=pos.Z(3:2:end);
pos.theta=pos.theta(3:2:end);
% time_vect_tf=zeros(length(scantf),1);
% 
% j = 0;
% for i=1:length(scantf)
%     time_vect_tf(i)=double(scantf{i}.transforms(1).header.stamp.sec)+double(scantf{i}.transforms(1).header.stamp.nanosec)*1e-9;
%     if i>1
%         if sum(time_vect_tf(i)==time_vect)>0 && time_vect_tf(i-1)~=time_vect_tf(i)
%             j = j+1;
%             struct_new{j} = scantf{i};
%         end
%     end
% end
% 
% scantf=struct_new';

% pos.X=zeros(N,1);
% pos.Y=zeros(N,1);
% pos.Z=zeros(N,1);
% pos.theta=zeros(N,1);
% 
% pos.time=zeros(N,1);
% 
% for ii=1:N
% 
%     pos.time(ii)=double(scanMsgs{ii}.header.stamp.sec)+double(scanMsgs{ii}.header.stamp.nanosec)*1e-9;
%     pos.theta(ii) = pos_robot.theta(pos_robot.time_vect==pos.time(ii));
%     pos.X(ii) = pos_robot.X(pos_robot.time_vect==pos.time(ii));
%     pos.Y(ii) = pos_robot.Y(pos_robot.time_vect==pos.time(ii));
%     pos.Z(ii) = pos_robot.Z(pos_robot.time_vect==pos.time(ii));
% 
% end

pos_glob.X= pos.X+scantf_static{1}.transforms(3).transform.translation.x-scantf_static{1}.transforms(4).transform.translation.x;
pos_glob.Y= pos.Y+scantf_static{1}.transforms(3).transform.translation.y-scantf_static{1}.transforms(4).transform.translation.y;
pos_glob.Z= pos.Z+scantf_static{1}.transforms(3).transform.translation.z-scantf_static{1}.transforms(4).transform.translation.z;
pos_glob.theta= unwrap(pos.theta);

% scan_new = scanMsgs;
for ii = 1:N
    % scan_new{ii}.ranges = scanMsgs{ii}.ranges;
    % % (pos_glob.Y(ii)*sin(pos_glob.theta(ii)) + pos_glob.X(ii)*cos(pos_glob.theta(ii)));
    % scan_new{ii}.ranges(scan_new{ii}.ranges<0) = inf;
    % ls = scan_new{ii};
    
    % legge dati scan
    sc = rosReadLidarScan(scanMsgs{ii});
    
    % prende frame del lidar e lo mette nel frame globale 
    sc_traslato = transformScan(sc, [pos_glob.X(ii), pos_glob.Y(ii), pos_glob.theta(ii)]);
    
    figure(656)
    linehandle = plot(sc_traslato); 
    linehandle.Color = [0 0 0]; 
    ylim([-3 4])
    xlim([-2 5])
    grid on; hold on
end
print(gcf,'scan_robot.eps', '-depsc', '-r300')
