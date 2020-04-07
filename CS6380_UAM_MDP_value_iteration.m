function [U,U_trace] = CS6380_UAM_MDP_value_iteration(S,A,P,Rfn,gamma,...
    eta,max_iter)
% CS6380_UAM_MDP_value_iteration - compute policy using value iteration
% On input:
%     S (vector): states (1 to n)
%     A (vector): actions (1 to k)
%     P (nxk array): transition model
%     Rfn (string): reward function: args are: state,action
%     gamma (float): discount factor
%     eta (float): termination threshold
%     max_iter (int): max number of iterations
% On output:
%     U (vector): state utilities
%     U_trace (iter x n): trace of utility values during iteration
% Call:
%     [U,Ut] = CS6380_UAM_MDP_value_iteration(S,A,P,Rfn,0.999999,0.1,100)
%
% Author:
%     T. Henderson
%     UU
%     Fall 2015; revised Spring 2020
%

dt = [];
num_states = length(S);
num_actions = length(A);
U = zeros(1,num_states);
U_p = U;

U_trace = [];

done = 0;
iter  = 0;
while done==0
    iter = iter + 1;
    U = U_p;
    delta = 0;
    for s = 1:num_states
        A_ind = 0;
        A_max = -Inf;
        for a = 1:num_actions
            val = 0;
            for sa = 1:num_states
                val = val + P(s,a).prob(sa)*U(sa);
            end
            if val>A_max
                A_ind = a;
                A_max = val;
            end
        end
        R = feval(Rfn,s,a);
        U_p(s) = R + gamma*A_max;
        if abs(U_p(s)-U(s))>delta
            delta = abs(U_p(s)-U(s));
        end
    end
    U_trace = [U_trace;U_p];
    dt = [dt;delta];
    if delta<eta*(1-gamma)/gamma||(iter>max_iter)
        done = 1;
    end
end
