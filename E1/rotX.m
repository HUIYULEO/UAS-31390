function [rotmtx] = rotX(angle)
% return the rotation matrix around x axis
rotmtx = [1, 0, 0; 0, cosd(angle), -sind(angle); 0, sind(angle), cosd(angle)];
end
