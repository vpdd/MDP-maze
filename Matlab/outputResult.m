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