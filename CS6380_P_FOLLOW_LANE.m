function [dx,dy,dz,speed] = CS6380_P_FOLLOW_LANE(loc,l_exit,del_t,...
    max_speed)
% CS6380_P_FOLLOW_LANE - produce actions to follow lane
% On input:
%     loc (1x3 vector): current location
%     l_exit (3x1 vector): lane exit point
%     del_t (float): delta time
%     max_speed (float): max UAS speed
% On output:
%     dx (float): x coord of heading
%     dy (float): y coord of heading
%     dz (float): z coord of heading
%     speed (float): desired speed
% Call:
%     [dx,dy,dz,speed] = CS56380_P_FOLLOW_LANE([xa;ya;za],way,0.1,10);
% Author:
%     T. Henderson
%     UU
%     Spring 2020
%

dir = l_exit - loc;
dir = dir/norm(dir);
dx = dir(1);
dy = dir(2);
dz = dir(3);
dist = norm(l_exit-loc);
speed = min(max_speed,dist/del_t);
