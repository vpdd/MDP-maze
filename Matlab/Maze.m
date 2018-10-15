function [] = Maze(mdp)

% initialize
if nargin < 1
    mdp = initilization();
end

% initialize the 1-horizon value function: V_1(s) = R(s), for each s \in S.
mdp.V(1,:) = mdp.R;

while mdp.flag == 0 && mdp.h < mdp.H
    mdp.V = [mdp.V; mdp.V(mdp.h-1, :)];      % create a new line (h-step) in ValueMatrix and update it!

    for s = 1:length(mdp.R)      
        if mdp.nonTerminalState(s) == 1     % do value fucntion only in non-terminal state
            [Q_max, act] = QLearning(s, mdp);
            mdp.V(mdp.h, s) = Q_max;
            mdp.policy(mdp.h, s) = act;
        end
    end
    mdp.flag = valueConvergence(mdp);  
    mdp.h = mdp.h + 1;
end

outputResult(mdp)

