function [] = main()
clc, clear

% we can customize mdp?parameters ahead, by calling "initilization()" or
%    other "initilization1()" functions like that

mdp = initilization();

Maze(mdp)
end



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




function [Q_max, act] = QLearning(s, mdp)

A = mdp.A;
V = mdp.V;
R = mdp.R;
gamma = mdp.gamma;
act = mdp.act;
h = mdp.h;

Q_max = -9999;

for a = 1:length(A)
    T = TransitionFun(A(a));
    %Q = T(s,:)*(repmat(R(s),length(R),1) + gamma * V(h-1,:)');
    Q = R(s) +  gamma*T(s,:)*V(h-1,:)';
    if Q > Q_max
        Q_max = Q;
        act = A(a);
    end
end


function [] = outputResult(mdp)

V = mdp.V;
policy = mdp.policy;

optPolicy = policy(size(policy,1),:);
optPolicy = [optPolicy(1:4); optPolicy(5:8); optPolicy(9:12)]

if mdp.h < mdp.H
    disp('The Value iteration converges!')
else
    disp('The Value iteration does not converge! Maximum iteration reaches!')
end

V_ = V;
V_(:,4) =[]; V_(:,5) =[]; V_(:,6) =[]; 
for i=1:size(V_,2)
    plot(1:size(V_,1), V_(:,i),'v-')
    hold on
end
hold off


function flag = valueConvergence(mdp)

V = mdp.V;
h = mdp.h;
epsilon = mdp.epsilon;

flag = 0;

% check whether Value reaches Equilibrium state.
if any(V(h,:)-V(h-1,:) >= epsilon)
    flag = 0;
else
    flag = 1;
end



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



function [T] = TransitionUncertainty(a)
%{
switch a
%     case '0'
%         % go 'North'
%         T = [0	0	0	0	0	0	0	0	0	0	0	0
%             0	0	0	0	0	0	0	0	0	0	0	0
%             0	0	0	0	0	0	0	0	0	0	0	0
%             0	0	0	1	0	0	0	0	0	0	0	0
%             0	0	0	0	0	0	0	0	0	0	0	0
%             0	0	0	0	0	0	0	0	0	0	0	0
%             0	0	0	0	0	0	0	0	0	0	0	0
%             0	0	0	0	0	0	0	0	0	0	0	0
%             0	0	0	0	0	0	0	0	0	0	0	0
%             0	0	0	0	0	0	0	0	0	0	0	0
%             0	0	0	0	0	0	0	0	0	0	0	0
%             0	0	0	0	0	0	0	0	0	0	0	0
%             ];
%         
    case 'N'
        % go 'North'
        T = [0.9	0.1	0	0	0	0	0	0	0	0	0	0
            0.1	0.8	0.1	0	0	0	0	0	0	0	0	0
            0	0.1	0.8	0.1	0	0	0	0	0	0	0	0
            0	0	0	1	0	0	0	0	0	0	0	0
            0.8	0	0	0	0.2	0	0	0	0	0	0	0
            0	0	0	0	0	0	0	0	0	0	0	0
            0	0	0.8	0	0	0	0.1	0.1	0	0	0	0
            0	0	0	0.8	0	0	0.1	0.1	0	0	0	0
            0	0	0	0	0.8	0	0	0	0.1	0.1	0	0
            0	0	0	0	0	0	0	0	0.1	0.8	0.1	0
            0	0	0	0	0	0	0.8	0	0	0.1	0	0.1
            0	0	0	0	0	0	0	0.8	0	0	0.1	0.1
            ];
    case 'W'
        % go 'West'
        T = [0.9	0	0	0	0.1	0	0	0	0	0	0	0
            0.8	0.2	0	0	0	0	0	0	0	0	0	0
            0	0.8	0.1	0	0	0	0.1	0	0	0	0	0
            0	0	0	1	0	0	0	0	0	0	0	0
            0.1	0	0	0	0.8	0	0	0	0.1	0	0	0
            0	0	0	0	0	0	0	0	0	0	0	0
            0	0	0.1	0	0	0	0.8	0	0	0	0.1	0
            0	0	0	0.1	0	0	0.8	0	0	0	0	0.1
            0	0	0	0	0.1	0	0	0	0.9	0	0	0
            0	0	0	0	0	0	0	0	0.8	0.2	0	0
            0	0	0	0	0	0	0.1	0	0	0.8	0.1	0
            0	0	0	0	0	0	0	0.1	0	0	0.8	0.1
            ];
    case 'S'
        % go 'South'
        T = [0.1	0.1	0	0	0.8	0	0	0	0	0	0	0
            0.1	0.8	0.1	0	0	0	0	0	0	0	0	0
            0	0.1	0	0.1	0	0	0.8	0	0	0	0	0
            0	0	0	1	0	0	0	0	0	0	0	0
            0	0	0	0	0.2	0	0	0	0.8	0	0	0
            0	0	0	0	0	0	0	0	0	0	0	0
            0	0	0	0	0	0	0.1	0.1	0	0	0.8	0
            0	0	0	0	0	0	0.1	0.1	0	0	0	0.8
            0	0	0	0	0	0	0	0	0.9	0.1	0	0
            0	0	0	0	0	0	0	0	0.1	0.8	0.1	0
            0	0	0	0	0	0	0	0	0	0.1	0.8	0.1
            0	0	0	0	0	0	0	0	0	0	0.1	0.9
            ];  
    case 'E'
        % go 'East'
        T = [0.1	0.8	0	0	0.1	0	0	0	0	0	0	0
            0	0.2	0.8	0	0	0	0	0	0	0	0	0
            0	0	0.1	0.8	0	0	0.1	0	0	0	0	0
            0	0	0	1	0	0	0	0	0	0	0	0
            0.1	0	0	0	0.8	0	0	0	0.1	0	0	0
            0	0	0	0	0	0	0	0	0	0	0	0
            0	0	0.1	0	0	0	0	0.8	0	0	0.1	0
            0	0	0	0.1	0	0	0	0.8	0	0	0	0.1
            0	0	0	0	0.1	0	0	0	0.1	0.8	0	0
            0	0	0	0	0	0	0	0	0	0.2	0.8	0
            0	0	0	0	0	0	0.1	0	0	0	0.1	0.8
            0	0	0	0	0	0	0	0.1	0	0	0	0.9
           ];  
%}

switch a
    case 'N'
        % go 'North'
        T = [0.9	0.1	0	0	0	0	0	0	0	0	0	0
            0.1	0.8	0.1	0	0	0	0	0	0	0	0	0
            0	0.1	0.8	0.1	0	0	0	0	0	0	0	0
            0	0	0	1	0	0	0	0	0	0	0	0
            0.8	0	0	0	0.2	0	0	0	0	0	0	0
            0	0	0	0	0	0	0	0	0	0	0	0
            0	0	0.8	0	0	0	0.1	0.1	0	0	0	0
            0	0	0	0	0	0	0	1	0	0	0	0
            0	0	0	0	0.8	0	0	0	0.1	0.1	0	0
            0	0	0	0	0	0	0	0	0.1	0.8	0.1	0
            0	0	0	0	0	0	0.8	0	0	0.1	0	0.1
            0	0	0	0	0	0	0	0.8	0	0	0.1	0.1
            ];
    case 'W'
        % go 'West'
        T = [0.9	0	0	0	0.1	0	0	0	0	0	0	0
            0.8	0.2	0	0	0	0	0	0	0	0	0	0
            0	0.8	0.1	0	0	0	0.1	0	0	0	0	0
            0	0	0	1	0	0	0	0	0	0	0	0
            0.1	0	0	0	0.8	0	0	0	0.1	0	0	0
            0	0	0	0	0	0	0	0	0	0	0	0
            0	0	0.1	0	0	0	0.8	0	0	0	0.1	0
            0	0	0	0	0	0	0	1	0	0	0	0
            0	0	0	0	0.1	0	0	0	0.9	0	0	0
            0	0	0	0	0	0	0	0	0.8	0.2	0	0
            0	0	0	0	0	0	0.1	0	0	0.8	0.1	0
            0	0	0	0	0	0	0	0.1	0	0	0.8	0.1
            ];
    case 'S'
        % go 'South'
        T = [0.1	0.1	0	0	0.8	0	0	0	0	0	0	0
            0.1	0.8	0.1	0	0	0	0	0	0	0	0	0
            0	0.1	0	0.1	0	0	0.8	0	0	0	0	0
            0	0	0	1	0	0	0	0	0	0	0	0
            0	0	0	0	0.2	0	0	0	0.8	0	0	0
            0	0	0	0	0	0	0	0	0	0	0	0
            0	0	0	0	0	0	0.1	0.1	0	0	0.8	0
            0	0	0	0	0	0	0	1	0	0	0	0
            0	0	0	0	0	0	0	0	0.9	0.1	0	0
            0	0	0	0	0	0	0	0	0.1	0.8	0.1	0
            0	0	0	0	0	0	0	0	0	0.1	0.8	0.1
            0	0	0	0	0	0	0	0	0	0	0.1	0.9
            ];  
    case 'E'
        % go 'East'
        T = [0.1	0.8	0	0	0.1	0	0	0	0	0	0	0
            0	0.2	0.8	0	0	0	0	0	0	0	0	0
            0	0	0.1	0.8	0	0	0.1	0	0	0	0	0
            0	0	0	1	0	0	0	0	0	0	0	0
            0.1	0	0	0	0.8	0	0	0	0.1	0	0	0
            0	0	0	0	0	0	0	0	0	0	0	0
            0	0	0.1	0	0	0	0	0.8	0	0	0.1	0
            0	0	0	0	0	0	0	1	0	0	0	0
            0	0	0	0	0.1	0	0	0	0.1	0.8	0	0
            0	0	0	0	0	0	0	0	0	0.2	0.8	0
            0	0	0	0	0	0	0.1	0	0	0	0.1	0.8
            0	0	0	0	0	0	0	0.1	0	0	0	0.9
            ]; 

end
