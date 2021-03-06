function P = CS6380_UAM_probs
% CS6380_UAM_probs - set the transition probabilities
% On input:
%     N/A
% On output:
%     P (transition prob struct): transition probabilities
%       (S,A).probs (1x8 vector): P(s'|S,A) for s' in {1,2,...,8}
% Call:
%     P = CS6380_UAM_probs;
% Author:
%     T. Henderson
%     UU
%     Spring 2020
%
% States:
%   1: ~N, ~R, ~W   -- assume this state has 0 probability
%   2: ~N, ~R,  W
%   3: ~N,  R, ~W
%   4: ~N,  R,  W
%   5:  N, ~R, ~W
%   6:  N, ~R,  W
%   7:  N,  R, ~W
%   8:  N,  R,  W

P(1,1).prob = zeros(1,8);
P(1,2).prob = zeros(1,8);
P(1,3).prob = zeros(1,8);
P(1,4).prob = zeros(1,8);

P(2,1).prob = [0.00,0.80,0.05,0.10,0.02,0.01,0.01,0.01];
P(2,2).prob = [0.00,0.75,0.05,0.10,0.02,0.06,0.01,0.01];
P(2,3).prob = [0.00,0.30,0.05,0.10,0.02,0.50,0.02,0.01];
P(2,4).prob = [0.00,0.20,0.05,0.10,0.02,0.60,0.02,0.01];

P(3,1).prob = [0.00,0.05,0.80,0.10,0.02,0.01,0.01,0.01];
P(3,2).prob = [0.00,0.05,0.21,0.10,0.02,0.01,0.50,0.01];
P(3,3).prob = [0.00,0.05,0.74,0.10,0.02,0.02,0.06,0.01];
P(3,4).prob = [0.00,0.05,0.20,0.10,0.02,0.02,0.60,0.01];

P(4,1).prob = [0.00,0.05,0.10,0.80,0.02,0.01,0.01,0.01];
P(4,2).prob = [0.00,0.05,0.10,0.70,0.02,0.01,0.11,0.01];
P(4,3).prob = [0.00,0.05,0.10,0.65,0.02,0.01,0.16,0.01];
P(4,4).prob = [0.00,0.05,0.10,0.30,0.02,0.01,0.51,0.01];

P(5,1).prob = [0.00,0.00,0.00,0.00,0.80,0.10,0.07,0.03];
P(5,2).prob = [0.00,0.00,0.00,0.00,0.80,0.10,0.07,0.03];
P(5,3).prob = [0.00,0.00,0.00,0.00,0.80,0.10,0.07,0.03];
P(5,4).prob = [0.00,0.00,0.00,0.00,0.80,0.10,0.07,0.03];

P(6,1).prob = [0.00,0.60,0.01,0.04,0.10,0.22,0.02,0.01];
P(6,2).prob = [0.00,0.50,0.01,0.02,0.05,0.40,0.01,0.01];
P(6,3).prob = [0.00,0.40,0.01,0.04,0.01,0.50,0.02,0.01];
P(6,4).prob = [0.00,0.10,0.01,0.02,0.05,0.8,0.010,0.01];

P(7,1).prob = [0.00,0.01,0.60,0.02,0.05,0.03,0.30,0.02];
P(7,2).prob = [0.00,0.01,0.20,0.01,0.03,0.03,0.70,0.02];
P(7,3).prob = [0.00,0.01,0.50,0.02,0.05,0.03,0.40,0.02];
P(7,4).prob = [0.00,0.01,0.15,0.01,0.03,0.03,0.75,0.02];

P(8,1).prob = [0.00,0.04,0.03,0.80,0.01,0.01,0.01,0.10];
P(8,2).prob = [0.00,0.04,0.03,0.70,0.01,0.01,0.01,0.20];
P(8,3).prob = [0.00,0.04,0.03,0.60,0.01,0.01,0.01,0.30];
P(8,4).prob = [0.00,0.04,0.03,0.20,0.01,0.01,0.01,0.70];
