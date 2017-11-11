function mdp = initilization()
% 1. parameter for iteration
mdp.V = [];
mdp.H = 100;                % total horizon
mdp.h = 2;                  % start horizion in loop with step-2 (or called horizion 2)
mdp.gamma = 1;              % discount factor
mdp.epsilon = 0.0001;       % terminal discrepancy
mdp.flag = 0;               % check whether the convergence reached, 0 is not! 


% 2. parameter of ACTION-SPACE
mdp.A = ['N', 'W', 'S', 'E'];
mdp.policy(1,:) = repmat(mdp.A(1),1,12);    % at horizion 1: set policy is 'N'
mdp.act = mdp.A(2);       


% 3. parameter of IMMEDIATE-REWARD. R in this demo is:
% R = [0 0   0 +1;
%      0 -999 0 -1;
%      0 0   0 0];
r = +1;         % the reward in "terminal state
pu = -1;        % the punishment in "non-terminal" state
ob = -999;      % the obstacles
fre = -0.04;    % the immediate reward in "non-terminal" states 
mdp.R = [fre, fre, fre, r, fre, ob, fre, pu, fre, fre, fre, fre];


% 4. find all non-terminal state in state
n = length(mdp.R);
nonTerminalState = zeros(1,n);
for i = 1:n
    if mdp.R(i) == fre
        nonTerminalState(i) = 1;    % non-terminal states   
    end
end   
mdp.nonTerminalState = nonTerminalState;