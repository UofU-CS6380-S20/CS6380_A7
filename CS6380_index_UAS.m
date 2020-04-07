function [UAS_out,index] = CS6380_index_UAS(UAS,UAS_name)
% CS6380_index_UAS - return index of UAS in table
% On input:
%     UAS (struct vector): UAS info
%     UAS_name (string): name of UAS
% On output:
%     UAS_out (struct vector): updated table
%     index (int): index of UAS_name in table
% Call:
%     [UAS,index] = CS6380_index_UAS(UAS,'UAS_tom_101');
% Author:
%     T. Henderson
%     UU
%     Spring 2020
%

UAS_out = UAS;

if isempty(UAS)
    UAS_out(1).name = UAS_name;
    UAS_out(1).domains = [];
    UAS_out(1).flights = [];
    index = 1;
    return
end

num_UAS = length(UAS);
for u = 1:num_UAS
    if strcmp(UAS(u).name,UAS_name)
        index = u;
        return
    end
end

num_UAS = num_UAS + 1;
UAS_out(num_UAS).name = UAS_name;
UAS_out(num_UAS).domains = [];
UAS_out(num_UAS).flights = [];
index = length(UAS_out);
