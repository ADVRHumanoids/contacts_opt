close all
clc

P1 = reshape(p_lift(4:end,1), 3, 3);
P1 = P1(1:2, :);
P1 = [P1 P1(:,1)];
P1_init = reshape(p_initial(4:end,1), 3, 3);
P1_init = P1_init(1:2, :);
P1_init = [P1_init P1_init(:,1)];
figure
plot(P1(1,:), P1(2,:), 'r', 'LineWidth', 1)
hold on
plot(P1_init(1,:), P1_init(2,:), 'k--', 'LineWidth', 1)
hold on
plot(com_lift(1,1), com_lift(2,1), 's');
xlabel('x','Interpreter','latex');
ylabel('y','Interpreter','latex');
set(gca,'TickLabelInterpreter','latex');grid on;
title('Front Left lift','Interpreter','latex');

P1 = reshape(p_lift([1:3, 7:end],2), 3, 3);
P1 = P1(1:2, :);
P1 = [P1 P1(:,1)];
P1_init = reshape(p_initial([1:3, 7:end],1), 3, 3);
P1_init = P1_init(1:2, :);
P1_init = [P1_init P1_init(:,1)];
figure
plot(P1(1,:), P1(2,:), 'r', 'LineWidth', 1)
hold on
plot(P1_init(1,:), P1_init(2,:), 'k--', 'LineWidth', 1)
hold on
plot(com_lift(1,2), com_lift(2,2), 's');
xlabel('x','Interpreter','latex');
ylabel('y','Interpreter','latex');
set(gca,'TickLabelInterpreter','latex');grid on;
title('Front Right lift','Interpreter','latex');
