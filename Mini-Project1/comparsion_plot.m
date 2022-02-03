%comparison_plot for SLAM

%Assume we have all the data being ploted
%load comparsion_data.mat

% figure();plot(t(2:end),vecnorm(q(:,2:N_step+1)-qhat_EKF(:,2:N_step+1)),...
%     t(2:end),vecnorm(q(:,2:N_step+1)-qhat_EKF1(:,2:N_step+1)),':o',...
%     t(2:end),vecnorm(q(:,2:N_step+1)-qhat_EKF2(:,2:N_step+1)),'--x',...
%     t(2:end),vecnorm(q(:,2:N_step+1)-qhat_EKF3(:,2:N_step+1)),'^','linewidth',2);
% 
% legend('wcov=0.04,vcov=0.07','wcov=0.05,vcov=0.1','wcov=0.06,vcov=0.15'...
%     ,'wcov=0.03,vcov=0.02');

figure();plot(t(2:48),vecnorm(q(:,2:48)-qhat(:,2:48)),...
    t(2:48),vecnorm(q(:,2:48)-qhat_NLS(:,2:48)),':o',...
    t(2:48),vecnorm(q(:,2:48)-qhat_EKF(:,2:48)),'--x','linewidth',2);

legend('direct','NLS','EKF');