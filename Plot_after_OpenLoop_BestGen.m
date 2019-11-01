Gen = (1:size(GenBest,1))';

[DMP, W, Am, Ym, tau, dt, time, Traj, F, init_pos, init_vel, T] = learn_rcp_batch(ts0);

time_raw = (0:56)'/100;

figure(3)
subplot(3,2,2)
scatter(Gen, Work_bestGen(:,end),35,'k','filled')
% scatter(Gen, [1201;847;Work_bestGen(3:end,1)],30,'k','filled')
xlabel('Generation')
ylabel('Optimal Cost of Transport (J/m)')
grid on


subplot(3,2,5)
scatter(Gen,100*Constraint_viol_indiv_bestGen(:,end),35,'r','filled')
% scatter(Gen,[Constraint_viol_indiv_bestGen(:,1), Constraint_viol_indiv_bestGen(:,6)],'*','*')
xlabel('Generation')
ylabel('Fore-aft CoM position error on treadmill (cm)')
% legend('Fore-aft CoM position error on treadmill', 'R_Hip_init - L_Hip_init')
grid on


subplot(3,2,3)
% scatter(Gen,Constraint_viol_indiv_bestGen(:,6),35,'r','filled')
% ylabel('Hip_{R-f} - Hip_{L-i} (deg)')

% scatter(Gen,[Constraint_viol_indiv_bestGen(:,1), Constraint_viol_indiv_bestGen(:,6)],'*','*')

scatter(Gen,abs(pitch_err_bestGen(:,end)),35,'r','filled')
ylabel('Pitch Error Norm (deg)')
xlabel('Generation')

grid on


subplot(3,2,1)
for i=1:5
scatter(Gen,[GenBest(:,i)],30,'filled')
hold on
end
ylim([.3,1.6])
xlabel('Generation')
ylabel('Decision Variables')
legend('\phi_0 normal', 'T', 'A_h', 'A_k', 'A_a')
grid on


subplot(3,2,4)
scatter(Gen,sqrt(Constraint_viol_indiv_bestGen(:,1,end).^2+Constraint_viol_indiv_bestGen(:,6,end).^2),35,'r','filled')
xlabel('Generation')
ylabel('Partial Constrt Violation (Hip & x_CoM Norm)')
grid on


subplot(3,2,6)
scatter(Gen,Constraint_viol_bestGen(:,end),35,'r','filled')
xlabel('Generation')
ylabel('Overall Constrt Violations Norm')

grid on




figure(4)
subplot(3,2,1)
plot(simout.tout,left_hip,'k','LineWidth',1.2)
hold on
plot(time_raw,-T(1:57,1),'--k','LineWidth',1.2)
ylabel('Hip Angle (deg)')
title('Left Leg')

subplot(3,2,2)
plot(simout.tout,right_hip,'k','LineWidth',1.2)
hold on
plot(time_raw,-T(1:57,4),'--k','LineWidth',1.2)
title('Right Leg')


subplot(3,2,3)
plot(simout.tout,left_knee,'k','LineWidth',1.2)
hold on
plot(time_raw,T(1:57,2),'--k','LineWidth',1.2)
ylabel('Knee Angle (deg)')


subplot(3,2,4)
plot(simout.tout,right_knee,'k','LineWidth',1.2)
hold on
plot(time_raw,T(1:57,5),'--k','LineWidth',1.2)



subplot(3,2,5)
plot(simout.tout,left_ankle,'k','LineWidth',1.2)
hold on
plot(time_raw,-T(1:57,3),'--k','LineWidth',1.2)
ylabel('Ankle Angle (deg)')
xlabel('Time (s)')


subplot(3,2,6)
plot(simout.tout,right_ankle,'k','LineWidth',1.2)
hold on
plot(time_raw,-T(1:57,6),'--k','LineWidth',1.2)
title('Right Leg')
xlabel('Time (s)')
legend('Optimal Trajectory','Demonstrated Trajectory')


figure (5)
subplot(2,1,1)
plot(simout.tout,100*pos,'k','LineWidth',1.2)
hold on
plot(simout.tout,pos(1,1)*ones(length(tout),1),'--k','LineWidth',1.0)
ylabel('Deviation from the treadmill center (cm)')
ylim([-20 10])

subplot(2,1,2)
plot(simout.tout,pitch,'k','LineWidth',1.2)
hold on
plot(simout.tout,pitch(1,1)*ones(length(tout),1),'--k','LineWidth',1.0)
xlabel('Time (s)')
ylabel('Torso Pitch (deg)')