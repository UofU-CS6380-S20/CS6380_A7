function speed_out = CS6380_P_CORRECT_SPEED(set_speed,speed)
% CS6380_P_CORRECT_SPEED - correct speed for UAS
% On input:
%     set_speed (float): desired speed
%     speed (float): actual ground speed
% On output:
%     speed_out (float): adjusted speed so as to get desired speed
% Call:
%     so = CS56380_P_CORRECT_SPEED(5,3);
% Author:
%     T. Henderson
%     UU
%     Spring 2020
%

if set_speed>speed
    speed_out = set_speed - 1;
elseif set_speed<speed
    speed_out = set_speed + 1;
else
    speed_out = set_speed;
end
