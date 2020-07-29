function [out] = isColision(pos)
%ISCOLISION Summary of this function goes here
%   Detailed explanation goes here
global obstacles

out = 0;
for i = 1:size(obstacles,1)
        obstacle = obstacles(i, :);
    dist = sqrt((pos(2)-obstacle(2))^2 + (pos(3)-obstacle(3))^2);
    if dist < obstacle(4)
        out = 1;
    end
end

