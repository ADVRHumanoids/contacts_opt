close all
clc

time=(0:length(F_ifopt_wheel_1)-1)*.01;

figure;
subplot(3,1,1);
plot(time,F_ifopt_wheel_1(1,:),'--'); ylabel('Fx','Interpreter','latex');xlabel('time','Interpreter','latex');
hold on
plot(time,F_wheel_1(1,:));
title('Front Left','Interpreter','latex')
legend('IPOPT','ForzaGiusta','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex');grid on
subplot(3,1,2);
plot(time,F_ifopt_wheel_1(2,:),'--'); ylabel('Fy','Interpreter','latex');xlabel('time','Interpreter','latex');
hold on
plot(time,F_wheel_1(2,:));
legend('IPOPT','ForzaGiusta','Interpreter','latex');
set(gca,'TickLabelInterpreter','latex');grid on
subplot(3,1,3);
plot(time,F_ifopt_wheel_1(3,:),'--'); ylabel('Fz','Interpreter','latex');xlabel('time','Interpreter','latex');
hold on
plot(time,F_wheel_1(3,:));
legend('IPOPT','ForzaGiusta','Interpreter','latex');
set(gca,'TickLabelInterpreter','latex');grid on


figure
subplot(3,1,1);
plot(time,F_ifopt_wheel_2(1,:),'--'); ylabel('Fx','Interpreter','latex');xlabel('time','Interpreter','latex');
hold on
plot(time,F_wheel_2(1,:));
title('Front Right','Interpreter','latex')
legend('IPOPT','ForzaGiusta','Interpreter','latex');
set(gca,'TickLabelInterpreter','latex');grid on
subplot(3,1,2);
plot(time,F_ifopt_wheel_2(2,:),'--'); ylabel('Fy','Interpreter','latex');xlabel('time','Interpreter','latex');
hold on
plot(time,F_wheel_2(2,:));
legend('IPOPT','ForzaGiusta','Interpreter','latex');
set(gca,'TickLabelInterpreter','latex');grid on
subplot(3,1,3);
plot(time,F_ifopt_wheel_2(3,:),'--'); ylabel('Fz','Interpreter','latex');xlabel('time','Interpreter','latex');
hold on
plot(time,F_wheel_2(3,:));
legend('IPOPT','ForzaGiusta','Interpreter','latex');
set(gca,'TickLabelInterpreter','latex');grid on

figure
subplot(3,1,1);
plot(time,F_ifopt_wheel_3(1,:),'--'); ylabel('Fx','Interpreter','latex');xlabel('time','Interpreter','latex');
hold on
plot(time,F_wheel_3(1,:));
title('Rear Left','Interpreter','latex')
legend('IPOPT','ForzaGiusta','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex');grid on
subplot(3,1,2);
plot(time,F_ifopt_wheel_3(2,:),'--');ylabel('Fy','Interpreter','latex');xlabel('time','Interpreter','latex');
hold on
plot(time,F_wheel_3(2,:));
legend('IPOPT','ForzaGiusta','Interpreter','latex');
set(gca,'TickLabelInterpreter','latex');grid on
subplot(3,1,3);
plot(time,F_ifopt_wheel_3(3,:),'--');ylabel('Fz','Interpreter','latex');xlabel('time','Interpreter','latex');
hold on
plot(time,F_wheel_3(3,:));
legend('IPOPT','ForzaGiusta','Interpreter','latex');
set(gca,'TickLabelInterpreter','latex');grid on

figure; 
subplot(3,1,1);
plot(time,F_ifopt_wheel_4(1,:),'--'); ylabel('Fx','Interpreter','latex');xlabel('time','Interpreter','latex');
hold on
plot(time,F_wheel_4(1,:));
title('Rear Right','Interpreter','latex')
legend('IPOPT','ForzaGiusta','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex');grid on
subplot(3,1,2);
plot(time,F_ifopt_wheel_4(2,:),'--'); ylabel('Fy','Interpreter','latex');xlabel('time','Interpreter','latex');
hold on
plot(time,F_wheel_4(2,:));
legend('IPOPT','ForzaGiusta','Interpreter','latex');
set(gca,'TickLabelInterpreter','latex');grid on
subplot(3,1,3);
plot(time,F_ifopt_wheel_4(3,:),'--'); ylabel('Fz','Interpreter','latex');xlabel('time','Interpreter','latex');
hold on
plot(time,F_wheel_4(3,:));
legend('IPOPT','ForzaGiusta','Interpreter','latex');
set(gca,'TickLabelInterpreter','latex');grid on