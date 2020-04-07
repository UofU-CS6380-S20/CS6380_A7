function [S,A,Rfn,P,U,Ut] = CS6380_run_UAM_value_iteration(gamma,max_iter)
% CS6380_run_UAM_value_iteration - set up UAM example and run it
% On input:
%     gamma (float): discount factor
%     max_iter (int): max iterations on utility computation
% On output:
%     S (1xn vector): states
%     A (1xm vector): actions
%     R (1xn vector): rewards at each state
%     P (nxm arracy struct): P(s,a) with field:
%       .probs (1xn vector): transition probabilities
%     U (1xn vector): state utilities
%     Ut (pxn array): trace of utilities over p steps
% Call:
%     [S,A,R,P,U,Ut] = CS6380_run_UAM_value_iteration(0.999999,100);
% Author:
%     T. Henderson
%     UU
%     Fall 2017; modified Spring 2020
%

S = [1:8];

% reward function: R:SxA->Reals
Rfn = 'CS6380_UAM_reward';

A = [1:4]; % Plans 1 to 5
%  Plan 1:  Heading <-- waypt-loc;  speed <- desired or exact
%  Plan 2:  Heading <-- waypt-loc;  speed <- adjust +/-
%  Plan 3:  Heading <-- (waypt+loc)/2 - loc;  desired or exact
%  Plan 4:  Heading <-- (waypt+loc)/2 - loc;  speed <- adjust

% Transition Probabilities
P = CS6380_UAM_probs;

eta = 0.1;
[U,Ut] = CS6380_UAM_MDP_value_iteration(S,A,P,Rfn,gamma,eta,max_iter);

%return
clf
plot(Ut);
