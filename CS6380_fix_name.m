function line_out = CS6380_fix_name(base,val,line_in)
% CS6380_fix_name - add val to base name (as string)
% On input:
%     base (string): e.g., 'UAS_tom_'  -- this should be the string
%                       in the input file
%     val (int): UAS number
%     line_in (string): line from file
% On output:
%     line_out (string): line-in with number added to base in line
% Call:
%     tline = CS6380_fix_name('UAS_tom_',100,...
%                 'function action = CS6380_UAS_tom_(percept)');
% Author:
%     T. Henderson
%     UU
%     Spring 2020
%

line_out = line_in;
if isempty(line_in)
    return
end

indexes = strfind(line_in,base);
if isempty(indexes)
    return
end

n = length(indexes);
k1 = 1;
tline = [];
for c = 1:n
    k = indexes(c);
    k2 = k + 7;
    tline = [tline,line_in(k1:k2),num2str(val)];
    if c==n
        tline = [tline,line_in(k2+1:end)];
    else
        tline = [tline,tline(k2+1:indexes(c)-1)];
        k1 = k2 + 1;
    end
end
line_out = tline;
