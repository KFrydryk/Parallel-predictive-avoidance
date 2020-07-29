clear all
close all
%global obstacles task_data
obstacle_data = jsondecode(fileread("obstacles1.json"));

LEFT_BOUNDARY = obstacle_data.left_boundary;
RIGHT_BOUNDARY = obstacle_data.right_boundary;
LOW_BOUNDARY = obstacle_data.low_boundary;
HIGH_BOUNDARY = obstacle_data.high_boundary;
MAX_RADIUS = obstacle_data.max_radius;
NUM_OF_OBSTACLES = obstacle_data.num_of_obstacles;
obstacles = cell2mat(struct2cell(obstacle_data.obstacles))';

% obstacles = create_obstackles(NUM_OF_OBSTACLES,...
%                                 LEFT_BOUNDARY,...
%                                 RIGHT_BOUNDARY,...
%                                 0,...
%                                 HIGH_BOUNDARY,...
%                                 MAX_RADIUS);
                            
%obstacles = {[20.5009032952522 65.1667257526498 2.73224719951534] [17.9776150476640 83.0186773226093 0.903688801273972] [15.1077348097610 4.10715493163691 4.61837806310204] [23.0739977801651 186.522714409713 0.817561842637628] [28.4219451178440 158.931577077751 2.88697098353324]}
task_data = jsondecode(fileread("task.json"));
targets = [pi/2, 20,200]';
startPos = [pi/2, 20, 0];

mdl = 'untitled1';
%load_system(mdl);
N = 120;
in(1:N) = Simulink.SimulationInput(mdl);
task_data.force.function = 1;


for i = 1:N
    in(i) = in(i).setBlockParameter('untitled1/obstacles', 'Value', mat2str(obstacles));
    in(i) = in(i).setBlockParameter('untitled1/targets', 'Value', mat2str(targets));
    
    %task_bus_info = Simulink.Bus.createObject(task_data);
    %task_bus = evalin('base', task_bus_info.busName);   
    %in(i) = in(i).setBlockParameter('untitled1/task_data', 'Value', task_data);
     in(i) = in(i).setBlockParameter('untitled1/td_vfo', 'Value', mat2str(cell2mat(struct2cell(task_data.vfo))));
     in(i) = in(i).setBlockParameter('untitled1/td_force', 'Value', mat2str(cell2mat(struct2cell(task_data.force))));
     in(i) = in(i).setBlockParameter('untitled1/td_results', 'Value', mat2str(cell2mat(struct2cell(task_data.results))));

    in(i) = in(i).setBlockParameter('untitled1/Unicycle/Integrator', 'InitialCondition', mat2str(startPos));
end
simOut = parsim(in)
% simOut = sim(mdl, 'SaveOutput', 'on',...
%     'OutputSaveName', 'yOut',...
%     'SaveTime', 'on',...
%     'TimeSaveName', 'tOut');

t = simOut(1).tout;
pos = simOut(1).q;
x = simOut(1).q.data(:,2);
y = simOut(1).q.data(:,3);
colisionDetected = 0;
if sum(simOut(1).colision.data) > 0
    colisionDetected = 1;
end

task_data.results.endTime = t(end);
task_data.results.colision = colisionDetected;

% fid = fopen('task.json', 'w');
% fprintf(fid, '%s', prettyjson(jsonencode(task_data)));
% fclose(fid);

axis([0 40 0 200]);
hold on;

%Draw boundaries
line([LEFT_BOUNDARY LEFT_BOUNDARY ], [0 HIGH_BOUNDARY]);
line([RIGHT_BOUNDARY RIGHT_BOUNDARY], [0 HIGH_BOUNDARY]);
for index = 1:NUM_OF_OBSTACLES
    vals = obstacles(index,:);
    %viscircles([vals(1), vals(2)], vals(3));
    drawObstacles([vals(2), vals(3), vals(4)]);
end
%plot x,y of robot
plot(x, y, 'r');

