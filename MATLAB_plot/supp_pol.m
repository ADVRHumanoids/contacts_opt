
P1 = reshape(p_lift(4:end,1), 3, 3);
P1 = P1(1:2, :);
P1 = [P1 P1(:,1)]
figure
plot(P1(1,:), P1(2,:), 'r', 'LineWidth', 2)
hold on
plot(com_lift(1,1), com_lift(2,1), 's');

P1 = reshape(p_lift([1:3, 7:end],2), 3, 3);
P1 = P1(1:2, :);
P1 = [P1 P1(:,1)]
figure
plot(P1(1,:), P1(2,:), 'r', 'LineWidth', 2)
hold on
plot(com_lift(1,2), com_lift(2,2), 's');

% P1 = reshape(p_lift([1:6, 10:end],3), 3, 3);
% P1 = P1(1:2, :);
% P1 = [P1 P1(:,1)]
% figure
% plot(P1(1,:), P1(2,:), 'r', 'LineWidth', 2)
% hold on
% plot(com_lift(1,3), com_lift(2,3), 's');
% 
% P1 = reshape(p_lift(1:9,4), 3, 3);
% P1 = P1(1:2, :);
% P1 = [P1 P1(:,1)]
% figure
% plot(P1(1,:), P1(2,:), 'r', 'LineWidth', 2)
% hold on
% plot(com_lift(1,4), com_lift(2,4), 's');