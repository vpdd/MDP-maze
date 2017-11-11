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
