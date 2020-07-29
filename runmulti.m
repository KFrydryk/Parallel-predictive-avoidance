clear all
close all
obstacle_data(1) = jsondecode(fileread("obstacles1.json"));
obstacle_data(2) = jsondecode(fileread("obstacles2.json"));
obstacle_data(3) = jsondecode(fileread("obstacles3.json"));
obstacle_data(4) = jsondecode(fileread("obstacles4.json"));
obstacle_data(5) = jsondecode(fileread("obstacles5.json"));

% acDist = 10;
% kval = 1;
final = [];
mdl = 'untitled1';
%load_system(mdl);

%continue from prev file saved

acDistMin  = 0.1;
acDistMax = 50.1;
acDistStep = 0.2;
kvalMin = 0.1;
kvalMax = 50.1;
kvalStep = 0.2;
stopFlag = false
for i = acDistMax : -1*acDistStep : acDistMin
    for j = kvalMax : -1*kvalStep : kvalMin
        if isfile("dumps/dump" + int2str(i*10) + "k" + int2str(j*10) + ".json")
            acDistMin = i+0.2;
            stopFlag = true
            break
        end
    end
    if stopFlag
        break
    end
end

%acDistIters = (acDistMax-acDistMin)/acDistStep + 1;
%kvalIters = (kvalMax-kvalMin)/kvalStep + 1;
%N = acDistIters*kvalIters*5;




for acDist = acDistMin : acDistStep : acDistMax
    
     N = 1255;
     Ni = 1;
     in(1:N) = Simulink.SimulationInput(mdl);
     results = repmat(struct('acDist', 1, 'kval', 1, 'obstacles', [1 1 1 1;2 2 2 2;3 3 3 3;4 4 4 4;5 5 5 5], 'endTime', 1, 'collision', 1),N,1);
        
    
    for kval = kvalMin : kvalStep : kvalMax
       
        for i = 1:5
            LEFT_BOUNDARY = obstacle_data(i).left_boundary;
            RIGHT_BOUNDARY = obstacle_data(i).right_boundary;
            LOW_BOUNDARY = obstacle_data(i).low_boundary;
            HIGH_BOUNDARY = obstacle_data(i).high_boundary;
            MAX_RADIUS = obstacle_data(i).max_radius;
            NUM_OF_OBSTACLES = obstacle_data(i).num_of_obstacles;
            %obstacles = struct2cell(obstacle_data(i).obstacles);
            obstacles = cell2mat(struct2cell(obstacle_data(i).obstacles))';
            %pause(0.1);
            task_data = jsondecode(fileread("task.json"));
            task_data.force.activationDistance = acDist;
            %task_data.force.function = int2str(kval) + "*sign/dist";
            task_data.force.function = 1;
            targets = [pi/2, 20,200]';
            startPos = [pi/2, 20, 0];
            
            in(Ni) = in(Ni).setBlockParameter('untitled1/obstacles', 'Value', mat2str(obstacles));
            in(Ni) = in(Ni).setBlockParameter('untitled1/targets', 'Value', mat2str(targets));
            in(Ni) = in(Ni).setBlockParameter('untitled1/td_vfo', 'Value', mat2str(cell2mat(struct2cell(task_data.vfo))));
            in(Ni) = in(Ni).setBlockParameter('untitled1/td_force', 'Value', mat2str(cell2mat(struct2cell(task_data.force))));
            in(Ni) = in(Ni).setBlockParameter('untitled1/td_results', 'Value', mat2str(cell2mat(struct2cell(task_data.results))));
            in(Ni) = in(Ni).setBlockParameter('untitled1/Unicycle/Integrator', 'InitialCondition', mat2str(startPos));
            
            results(Ni).acDist = acDist;
            results(Ni).kval = kval;
            results(Ni).obstacles = obstacles;
            
            Ni = Ni+1;
%              disp("AcDist: " + int2str(acDist*10) + ", Kval: " + int2str(kval*10))
%             results = struct('acDist', acDist, 'kval', kval, 'obstacles', obstacles, 'endTime', t(end), 'collision', colisionDetected);
%             final = [final, results];
        %     fid = fopen('task.json', 'w');
        %     fprintf(fid, '%s', prettyjson(jsonencode(task_data)));
        %     fclose(fid);
        end
    end
    
    simOut = parsim(in, 'ShowProgress', 'on', 'ShowSimulationManager','on', 'UseFastRestart', 'on')
    
    for i = 1:1:N
    t = simOut(i).tout;
    %pos = simOut(i).q;
    %x = simOut(i).q.data(:,2);
    %y = simOut(i).q.data(:,3);
    colisionDetected = 0;
    if sum(simOut(i).colision.data) > 0
        colisionDetected = 1;
    end
    
    results(i).endTime = t(end);
    results(i).collision = colisionDetected;
    end
    
    
     disp("AcDist: " + int2str(acDist*10) + ", Kval: " + int2str(kval*10))
     name = "dumps/dump" + int2str(acDist*10) + "k" + int2str(kval*10) + ".json";
     fid = fopen(name, 'wt');
     fprintf(fid, '%s', prettyjson(jsonencode(results)));
     fclose(fid);
end 







% fid = fopen('finalresults.json', 'wt');
% fprintf(fid, '%s', prettyjson(jsonencode(results)));
% fclose(fid);