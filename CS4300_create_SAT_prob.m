function CS4300_create_SAT_prob(KB,fn)
%

MINUS = '~';
BLANK = ' ';
x = 'x';

fd = fopen(fn,'w');
num_clauses = length(KB);

for c = 1:num_clauses
    clause = KB(c).clauses;
    len_clause = length(clause);
    for d = 1:len_clause
        if clause(d)<0
            fprintf(fd,'%s',MINUS);
        end
        fprintf(fd,'%s',x);
        fprintf(fd,'%d',abs(clause(d)));
        fprintf(fd,'%s',BLANK);
    end
    fprintf(fd,'\n');
end
fclose(fd);
