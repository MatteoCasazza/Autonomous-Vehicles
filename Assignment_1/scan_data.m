clc
clear
close all

%% BAG: select the bag and the /scan topic
bagFolder = "tb3_nocmd"; % folder with metadata.yaml + *.mcap
bag = ros2bagreader(bagFolder);
% Select a TOPIC (/scan)
sel = select(bag, "Topic", "/scan");
% (additional) Limit in time:
t0 = sel.StartTime; t1 = sel.EndTime;
sel = select(sel, "Time", [t0 t1]);
% read messages as struct
scanMsgs = readMessages(sel);

bagFolder = "tb3_nocmd"; % folder with metadata.yaml + *.mcap
bag = ros2bagreader(bagFolder);
% Select a TOPIC (/scan)
sel = select(bag, "Topic", "/tf");
% (additional) Limit in time:
t0 = sel.StartTime; t1 = sel.EndTime;
sel = select(sel, "Time", [t0 t1]);
% read messages as struct
scantf = readMessages(sel);

bagFolder = "tb3_nocmd"; % folder with metadata.yaml + *.mcap
bag = ros2bagreader(bagFolder);
% Select a TOPIC (/scan)
sel = select(bag, "Topic", "/tf_static");
% (additional) Limit in time:
t0 = sel.StartTime; t1 = sel.EndTime;
sel = select(sel, "Time", [t0 t1]);
% read messages as struct
scantf_static = readMessages(sel);