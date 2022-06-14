function [rotmtx] = rotZ(angle)
% return the rotation matrix around x axis
rotmtx = [cosd(angle), -sind(angle), 0; sind(angle), cosd(angle), 0; 0, 0, 1];
end