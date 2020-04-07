function theta = CS6380_angle_between(dir1,dir2)
% CS6380_angle_between - return smaller angle between two vectors
% On input:
%     dir1 (length k vector): direction 1
%     dir2 (length k vector): direction 2
% On output:
%     theta (float): angle (radians) between directions
% Call:
%     t = Cs6380_angle_between([1,0],[0,1]);
% Author:
%     T. Henderson
%     UU
%     Spring 2020
%

theta = 0;
if sum(dir1)==0|sum(dir2)==0
    return
end
theta = acos(dot(dir1,dir2)/(norm(dir1)*norm(dir2)));
