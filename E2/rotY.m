function [rotmtx] = rotY(angle)
% return the rotation matrix around x axis
rotmtx = [cosd(angle), 0, sind(angle); 0, 1, 0; -sind(angle), 0, cosd(angle)];
end