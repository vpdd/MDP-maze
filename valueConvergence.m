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