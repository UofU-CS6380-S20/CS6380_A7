function [mag,dir] = CS6380_wind(x,y,z,t)
% CS6380_wind - gets wind direction and magnitude location and time 
% On input:
%     x (float): x coord
%     y (float): y coord
%     z (float): z coord
%     t (float): time
% On output:
%     mag (float): magnitude of wind
%     dir (3x1 vector): direction vector of wind
% Call:
%     [mag,dir] = CS6380_wind(40,40,10,20,13);
% Author:
%     T. Henderson
%     UU
%     Spring 2020
%

mag = 0;
dir = [0;0;1];
CS6380_load_wind_data;

if isempty(centers)
    return
end
[num_centers,dummy] = size(centers);
v = centers(1,5)*centers(1,6:8)';
for c = 2:num_centers
    v = v + centers(c,5)*centers(c,6:8)';
end
mag = norm(v);
dir = v/mag;
