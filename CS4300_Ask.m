function b = CS4300_Ask(KB,sentence)
% CS4300_Ask - Ask function for logic KB
% On input:
%     KB (KB struct): Knowledge base (CNF)
%       (k).clauses (1xp vector): disjunction clause
%     sentence (KB struct): query theorem (CNF)
%       (k).clauses (1xq vector): disjunction
% On output:
%     b (Boolean): 1 if KB entails sentence, else 0
% Call:
%     KB(1).clauses = [1];
%     KB(2).clauses = [-1,2];
%     sentence(1).clauses = [2];
%     b = CS4300_Ask(KB,sentence);
% Author:
%     T. Henderson
%     UU
%     Fall 2017
%

b = 0;

if isempty(sentence)
    return
end

vars = CS4300_vars(KB,sentence);
num_sentences = length(KB);
len_sentence = length(sentence);
for s = 1:len_sentence
    KB(num_sentences+1).clauses = -sentence(s).clauses;
    CS4300_create_SAT_prob(KB,'HYBKB');
    system('sat.py < HYBKB >popoo');
    fd = fopen('popoo','r');
    t = fscanf(fd,'%s');
    if ~isempty(t)
        fclose(fd);
        return
    end
    clear t
    fclose(fd);
end

b = 1;
