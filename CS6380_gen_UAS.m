function CS6380_gen_UAS(base,range,fn)
% CS6380_gen_UAS - generate a set of UAS .m files
% On input:
%     base (string): e.g., 'UAS_tom_'  -- this should be the string
%                       in the input file
%     range (1xm vector): number range for UAS files (e.g., [100:110]
%     fn (string): input template filename
% On output:
%     writes a set of files with names base<range>.m
% Call:
%     CS6380_gen_UAS('UAS_tom_',[10:11],'test1.txt');
% Author:
%     T. Henderson
%     UU
%     Spring 2020
%

if isempty(range)
    return
end

num_copies = length(range);
for c = 1:num_copies
    val = range(c);
    fd_in = fopen(fn,'r');
    fn_out = ['CS6380_',base,num2str(val),'.m'];
    fd_out = fopen(fn_out,'w');
    tline = fgetl(fd_in);
    while ischar(tline)
        tline = CS6380_fix_name(base,val,tline);
        fprintf(fd_out,'%s\n',tline);
        tline = fgetl(fd_in);
    end
    fclose(fd_out);
end
fclose(fd_in);
