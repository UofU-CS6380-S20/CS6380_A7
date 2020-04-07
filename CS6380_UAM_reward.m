function r = CS6380_UAM_reward(s,a)
% CS6380_UAM_reward - determine reward for state - action pair
% On input:
%     s (int): state index
%     a (int): action index
% On output:
%     r (float): reward for state action
% Call:
%     P = CS6380_UAM_reward(5,1);
% Author:
%     T. Henderson
%     UU
%     Spring 2020
%

state_rewards = [-2,-2,-2,-2,6,6,6,6];

action_rewards = [-1,-3,-5,-8];

r = state_rewards(s) + action_rewards(a);
