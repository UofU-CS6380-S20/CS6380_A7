function [res,M] = CS6380_drive_A7_ABMS(max_t,draw,film,dump)
% CS6380_drive_A7_ABMS - overall driver for A7 ABMS
% On input:
%     max_t (float): max simulation time
%     draw (Boolean): draw during simulation
%     film (Boolean): make a movie (not implemented)
% On output:
%     res (struct vector): results 
% Call:
%     r1 = CS6380_drive_A7_ABMS(100,1,0,0);
% Author:
%     T. Henderson
%     UU
%     Spring 2020
%

global g_fig

res = [];

ports = [];
x_vals = [0,10,20,30,40];
y_vals = x_vals;
num_x_vals = length(x_vals);
num_y_vals = length(y_vals);
for indx = 1:num_x_vals
    x = x_vals(indx);
    for indy = 1:num_y_vals
        y = y_vals(indy);
        ports = [ports; x,y];
    end
end

del_t = 0.1;
fnames(1).name = 'CS6380_USS_tom_1';
fnames(2).name = 'CS6380_GRS_tom_1';
fnames(3).name = 'CS6380_ATOC_tom_1';
fnames(4).name = 'CS6380_UAS_tom_100';
fnames(5).name = 'CS6380_UAS_tom_101';
fnames(6).name = 'CS6380_UAS_tom_102';
fnames(7).name = 'CS6380_UAS_tom_103';
fnames(8).name = 'CS6380_UAS_tom_104';
fnames(9).name = 'CS6380_UAS_tom_105';
fnames(10).name = 'CS6380_UAS_tom_106';
fnames(11).name = 'CS6380_UAS_tom_107';
fnames(12).name = 'CS6380_UAS_tom_108';
fnames(13).name = 'CS6380_UAS_tom_109';

% clear persistent variables
num_agents = length(fnames);
for a = 1:num_agents
    clear(fnames(a).name);
end
clear('CS6380_A4_ABMS');

[res,M] = CS6380_A7_ABMS(fnames,ports,max_t,del_t,draw,film,dump);

close(g_fig);
