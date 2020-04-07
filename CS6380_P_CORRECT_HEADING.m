function [dx,dy,dz] = CS6380_P_CORRECT_HEADING(loc,pt2)
% CS6380_P_CORRECT_HEADING - correct heading for UAS
% On input:
%     loc (3x1 vector): current location
%     pt2 (3x1 vector): desired place to go
% On output:
%     dx (float): x coordinate of heading
%     dy (float): y coordinate of heading
%     dz (float): z coordinate of heading
% Call:
%     so = CS56380_P_CORRECT_HEADING([xa;ya;za],way);
% Author:
%     T. Henderson
%     UU
%     Spring 2020
%

dir = pt2 - loc;
dir = dir/norm(dir);
dx = dir(1);
dy = dir(2);
dz = dir(3);
