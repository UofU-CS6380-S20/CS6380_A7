function KB_out = CS4300_Tell(KB,sentence)
% CS4300_Tell - add sentence to KB
% On input:
%     KB (nx1 struct vector): knowledge base with fields:
%       (c).clauses (1xk vector): disjunction
%     sentence (mx1 struct vector): true sentence to add to KB
% On output:
%     KB_out (px1 struct vector): updated KB
% Call:
%     KB = CS4300_Tell(KB,s);
% Author:
%     T. Henderson
%     UU
%     Fall 2017
%

KB_out = KB;
len_sentence = length(sentence);
for s = 1:len_sentence
    clause1 = sort(sentence(s).clauses);
    found_same = 0;
    found_opposite = 0;
    for n = 1:length(KB_out)
        clause2 = sort(KB_out(n).clauses);
        if length(clause1)==length(clause2)
            if sum(clause1-clause2)==0
                found_same = 1;
            elseif sum(clause1+clause2)==0
                found_opposite = n;
            end
        end
    end
    if found_opposite~=0
        KB_out(found_opposite).clauses = clause1;
    elseif found_same==0
        KB_out(end+1).clauses = clause1;
    end
end
