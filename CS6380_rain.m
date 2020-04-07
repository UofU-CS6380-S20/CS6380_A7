function intensity = CS6380_rain(x,y,z,t)
% CS6380_rain - gets rain intensity at location and time 
% On input:
%     x (float): x coord
%     y (float): y coord
%     z (float): z coord
%     t (float): time
% On output:
%     intensity (float): intensity of rain (in [0,1] interval)
% Call:
%     r = CS6380_rain(40,40,10,20,13);
% Author:
%     T. Henderson
%     UU
%     Spring 2020
%

intensity = 0;
CS6380_load_rain_data;

if isempty(centers)
    return
end

[num_centers,dummy] = size(centers);
for c = 1:num_centers
    if norm([x;y;z]-centers(c,1:3)')<=centers(c,4)
        intensity = max(intensity,centers(c,5));
    end
end
