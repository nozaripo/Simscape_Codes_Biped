
robotParameters;

youBot_PARAM;

pFinal=GenBest(10,:)




for j = 1:6
%     GenBest_vel(:,:,k) = GenBest'
V_treadmill = .3+(j-1)*1.7/5;
    j
    [penalty_bestGen(j,k), Work_bestGen(j,k), pitch_err, Constraint_viol, Constraint_viol_indiv] = CostFCN(GenBest(j,:),mdlName,V_treadmill);
    Constraint_viol_bestGen(j,k)=Constraint_viol;
    pitch_err_bestGen(j,k)=pitch_err;
    Constraint_viol_indiv_bestGen(j,:,k)=Constraint_viol_indiv;
%     init_ht_sym(j,1) = z_lh(1,1)-z_rt(1,1);
%     final_stop(j,1)  = z_rh(1,end)-5.01;
    clear z_lh z_rt z_lt z_rh
end
pFinal(1,1)=35+7*pFinal(1,1);
ts0 = ceil(pFinal(1,1));    
%     ts0 = pFinal(1,1); 
% ts0 = 43;
    tau = pFinal(1,2);
    
  
    
%     theta0 = pFinal(1,4);
    
    tau_nom=1.13;
    tSim = (.57/tau_nom)*tau;

[DMP, W, ~, Ym, tau_nom, dt, time, Tra, F, init_pos, init_vel] = learn_rcp_batch(ts0);

Am = pFinal(1,3:end);

% Am(1,3) = 0.7;

Am = repmat(Am,1,6/length(Am));

Traj = Am.*init_pos'*pi/180;

Ym = Ym.*Am;

init_pos = Am'.*init_pos;
init_vel = Am'.*init_vel;




% theta0 = params(1,4);


tSim = (.57/tau_nom)*tau;




options = optimoptions('fsolve','Display','off','FunctionTolerance',1e-10,'MaxFunctionEvaluations',1000);

q0 = 0;

fun = @(q)torso_pitch(q, Traj);
    [q, fval] = fsolve(fun,q0,options)
    

theta0 = q;
[~, init_h, init_hR] = torso_pitch(theta0,Traj)

init_height = init_h;

% init_height = max(12.5*sin(-Traj(1,1)-theta0+Traj(1,2)-Traj(1,3)) , -4.5*sin(-Traj(1,1)-theta0+Traj(1,2)-Traj(1,3))) + ...
%     38*( cos(-Traj(1,1)-theta0) + cos(-Traj(1,1)-theta0+Traj(1,2)) ) + 3*cos(-Traj(1,1)-theta0+Traj(1,2)-Traj(1,3));
% 
% 
% init_height = max(-12.5*sin(Traj(1,1)+theta0-Traj(1,2)+Traj(1,3)) , +4.5*sin(Traj(1,1)+theta0-Traj(1,2)+Traj(1,3))) + ...
%     38*( cos(Traj(1,1)+theta0) + cos(Traj(1,1)+theta0-Traj(1,2)) ) + 3*cos(Traj(1,1)+theta0-Traj(1,2)+Traj(1,3));


% init_height = max(12.5*sin(-Traj(1,1)-theta0+Traj(1,2)-Traj(1,3)) , -4.5*sin(-Traj(1,1)-theta0+Traj(1,2)-Traj(1,3))) + ...
%     38*( cos(-Traj(1,1)-theta0) + cos(-Traj(1,1)-theta0+Traj(1,2)) ) + 3*cos(-Traj(1,1)-theta0+Traj(1,2)-Traj(1,3)) +.2 ;




% simout = sim(mdlName,'SrcWorkspace','current');






function [F, Z_L, Z_R] = torso_pitch(q, Traj)

theta0 = q;


Z_L = max(-12.5*sin(Traj(1,1)+theta0-Traj(1,2)+Traj(1,3)) , +4.5*sin(Traj(1,1)+theta0-Traj(1,2)+Traj(1,3))) + ...
    38*( cos(Traj(1,1)+theta0) + cos(Traj(1,1)+theta0-Traj(1,2)) ) + 4*cos(Traj(1,1)+theta0-Traj(1,2)+Traj(1,3));

Z_R = max(12.5*sin(-Traj(1,4)-theta0 +Traj(1,5)-Traj(1,6)) , -4.5*sin(-Traj(1,4)-theta0 +Traj(1,5)-Traj(1,6)))+...
    38*( cos(-Traj(1,4)-theta0) + cos(-Traj(1,4)-theta0+Traj(1,5)) ) + 4*cos(-Traj(1,4)-theta0 +Traj(1,5)-Traj(1,6));

% Z_L = 4.5*sin(Traj(1,1)+theta0-Traj(1,2)+Traj(1,3)) + ...
%     38*( cos(Traj(1,1)+theta0) + cos(Traj(1,1)+theta0-Traj(1,2)) ) + 4*cos(Traj(1,1)+theta0-Traj(1,2)+Traj(1,3));
% 
% Z_R = 12.5*sin(-Traj(1,4)-theta0 +Traj(1,5)-Traj(1,6))+...
%     38*( cos(-Traj(1,4)-theta0) + cos(-Traj(1,4)-theta0+Traj(1,5)) ) + 4*cos(-Traj(1,4)-theta0 +Traj(1,5)-Traj(1,6));


F(1) = Z_L-Z_R;

end