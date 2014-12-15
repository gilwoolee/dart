clear all;
close all;

angular_terms = load('angular.txt');
t = angular_terms(:,1)';
ang_vel = angular_terms(:,2)';
ang_mom = angular_terms(:,3)';
ang_mom_diff = diff(ang_mom);

figure(1);
subplot(3, 1, 1); % subplot
plot(t, ang_vel);
grid on
xlabel('time')
ylabel('norm')
legend('Angular velocity')

subplot(3, 1, 2); % subplot
plot(t, ang_mom);
grid on
xlabel('time')
ylabel('norm')
legend('Angular momentum')

subplot(3, 1, 3); % subplot
plot(t(1:end-1), ang_mom_diff);
grid on
xlabel('time')
ylabel('norm')
legend('Angular momentum diff')