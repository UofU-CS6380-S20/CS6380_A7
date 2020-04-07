function goals = CS6380_set_goals_truth(goals_in,KB)
%

goals = goals_in;

num_goals = length(goals(:,1));
for g = 1:num_goals
    if CS6380_Ask_clause(KB,[goals(g,1)])
        goals(g,2) = 1;
    else
        goals(g,2) = 0;
    end
end
