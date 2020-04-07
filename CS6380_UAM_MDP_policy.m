function policy = CS6380_UAM_MDP_policy(S,A,P,U)
% CS6380_UAM_MDP_policy - generate a policy from utilities
% See p. 648 Russell & Norvig
% On input:
%     S (vector): states (1 to n)
%     A (vector): actions (1 to k)
%     P (nxk array): transition model
%     U (vector): state utilities
% On output:
%     policy (vector): actions per state
% Call:
%     p = CS6380_UAM_MDP_policy(S,A,P,U);
% Author:
%     T. Henderson
%     UU
%     Fall 2014; modified Spring 2020
%

num_actions = length(A);
num_states = length(S);
policy = zeros(1,num_states);

for s = 1:num_states
    A_max = -Inf;
    A_index = 0;
    for a = 1:num_actions
        val = 0;
        for sa = 1:num_states
            val = val + P(s,a).prob(sa)*U(sa);
        end
        if val>A_max
            A_max = val;
            A_index = a;
        end
    end
    policy(s) = A_index;
end
