% PLOT IFOPT

close all;
clc;

% addpath('superquadratics');

com = com_final;
x = x_sol_final;

Fd = F_ref + [0 0 1000]';

[x_s,y_s,z_s] = sphere;
[x_e, y_e, z_e]=superellipsoid(C,R,P,100);

fig=figure;
fig.Units='centimeters';
fig.Position=[10 5 15 25];
subplot(3,1,1)
plot3([x(1),com(1)],[x(2),com(2)],[x(3),com(3)],'Color',[0    0.4470    0.7410])
hold on
plot3([x(4),com(1)],[x(5),com(2)],[x(6),com(3)],'Color',[0    0.4470    0.7410])
hold on
plot3([x(7),com(1)],[x(8),com(2)],[x(9),com(3)],'Color',[0    0.4470    0.7410])
hold on
plot3([x(10),com(1)],[x(11),com(2)],[x(12),com(3)],'Color',[0    0.4470    0.7410])
hold on
scatter3(x(1),x(2),x(3),'MarkerEdgeColor','k','MarkerFaceColor',[0.8500    0.3250    0.0980]);
hold on
scatter3(x(4),x(5),x(6),'MarkerEdgeColor','k','MarkerFaceColor',[0.8500    0.3250    0.0980]);
hold on
scatter3(x(7),x(8),x(9),'MarkerEdgeColor','k','MarkerFaceColor',[0.8500    0.3250    0.0980]);
hold on
scatter3(x(10),x(11),x(12),'MarkerEdgeColor','k','MarkerFaceColor',[0.8500    0.3250    0.0980]);
hold on
surf(x_s*.05+com(1),y_s*.05+com(2),z_s*.05+com(3), 'FaceColor', [0    0.4470    0.7410],'edgecolor','none');
hold on;
h=surf(x_e,y_e,z_e);
set(h,'FaceColor', [.8 .8 .8],'edgecolor','none'); alpha(h,.6);
hold on;
vectarrow(x(1:3),x(1:3) + x(13:15)/norm(x(13:15))*.5);
hold on;
vectarrow(x(4:6),x(4:6) + x(16:18)/norm(x(16:18))*.5);
hold on;
vectarrow(x(7:9),x(7:9) + x(19:21)/norm(x(19:21))*.5);
hold on;
vectarrow(x(10:12),x(10:12) + x(22:24)/norm(x(22:24))*.5);
hold on;
vectarrow(com',com' + Fd'/norm(Fd)*.5);
grid on;
xlabel('$x$ axis','Interpreter','latex');
ylabel('$y$ axis','Interpreter','latex');
zlabel('$z$ axis','Interpreter','latex');
xlim([-1 1]);
ylim([-0.5 0.5]);
zlim([0 1]);
set(gca,'TickLabelInterpreter','latex');
view([22 24])

subplot(3,1,2)
plot3([x(1),com(1)],[x(2),com(2)],[x(3),com(3)],'Color',[0    0.4470    0.7410])
hold on
plot3([x(4),com(1)],[x(5),com(2)],[x(6),com(3)],'Color',[0    0.4470    0.7410])
hold on
plot3([x(7),com(1)],[x(8),com(2)],[x(9),com(3)],'Color',[0    0.4470    0.7410])
hold on
plot3([x(10),com(1)],[x(11),com(2)],[x(12),com(3)],'Color',[0    0.4470    0.7410])
hold on
scatter3(x(1),x(2),x(3),'MarkerEdgeColor','k','MarkerFaceColor',[0.8500    0.3250    0.0980]);
hold on
scatter3(x(4),x(5),x(6),'MarkerEdgeColor','k','MarkerFaceColor',[0.8500    0.3250    0.0980]);
hold on
scatter3(x(7),x(8),x(9),'MarkerEdgeColor','k','MarkerFaceColor',[0.8500    0.3250    0.0980]);
hold on
scatter3(x(10),x(11),x(12),'MarkerEdgeColor','k','MarkerFaceColor',[0.8500    0.3250    0.0980]);
hold on
surf(x_s*.05+com(1),y_s*.05+com(2),z_s*.05+com(3), 'FaceColor', [0    0.4470    0.7410],'edgecolor','none');
hold on;
h=surf(x_e,y_e,z_e);
set(h,'FaceColor', [.8 .8 .8],'edgecolor','none'); alpha(h,.6);
hold on;
vectarrow(x(1:3),x(1:3) + x(13:15)/norm(x(13:15))*.5);
hold on;
vectarrow(x(4:6),x(4:6) + x(16:18)/norm(x(16:18))*.5);
hold on;
vectarrow(x(7:9),x(7:9) + x(19:21)/norm(x(19:21))*.5);
hold on;
vectarrow(x(10:12),x(10:12) + x(22:24)/norm(x(22:24))*.5);
hold on;
vectarrow(com',com' + Fd'/norm(Fd)*.5);
grid on;
xlabel('$x$ axis','Interpreter','latex');
ylabel('$y$ axis','Interpreter','latex');
zlabel('$z$ axis','Interpreter','latex');
xlim([-1 1]);
ylim([-0.5 0.5]);
zlim([0 1]);
set(gca,'TickLabelInterpreter','latex');
view([0 90])

subplot(3,1,3)
plot3([x(1),com(1)],[x(2),com(2)],[x(3),com(3)],'Color',[0    0.4470    0.7410])
hold on
plot3([x(4),com(1)],[x(5),com(2)],[x(6),com(3)],'Color',[0    0.4470    0.7410])
hold on
plot3([x(7),com(1)],[x(8),com(2)],[x(9),com(3)],'Color',[0    0.4470    0.7410])
hold on
plot3([x(10),com(1)],[x(11),com(2)],[x(12),com(3)],'Color',[0    0.4470    0.7410])
hold on
scatter3(x(1),x(2),x(3),'MarkerEdgeColor','k','MarkerFaceColor',[0.8500    0.3250    0.0980]);
hold on
scatter3(x(4),x(5),x(6),'MarkerEdgeColor','k','MarkerFaceColor',[0.8500    0.3250    0.0980]);
hold on
scatter3(x(7),x(8),x(9),'MarkerEdgeColor','k','MarkerFaceColor',[0.8500    0.3250    0.0980]);
hold on
scatter3(x(10),x(11),x(12),'MarkerEdgeColor','k','MarkerFaceColor',[0.8500    0.3250    0.0980]);
hold on
surf(x_s*.05+com(1),y_s*.05+com(2),z_s*.05+com(3), 'FaceColor', [0    0.4470    0.7410],'edgecolor','none');
hold on;
h=surf(x_e,y_e,z_e);
set(h,'FaceColor', [.5 .5 .5],'edgecolor','none'); alpha(h,1);
hold on;
vectarrow(x(1:3),x(1:3) + x(13:15)/norm(x(13:15))*.5);
hold on;
vectarrow(x(4:6),x(4:6) + x(16:18)/norm(x(16:18))*.5);
hold on;
vectarrow(x(7:9),x(7:9) + x(19:21)/norm(x(19:21))*.5);
hold on;
vectarrow(x(10:12),x(10:12) + x(22:24)/norm(x(22:24))*.5);
hold on;
vectarrow(com',com' + Fd'/norm(Fd)*.5);
grid on;
xlabel('$x$ axis','Interpreter','latex');
ylabel('$y$ axis','Interpreter','latex');
zlabel('$z$ axis','Interpreter','latex');
xlim([-1 1]);
ylim([-0.5 0.5]);
zlim([0 1.5]);
set(gca,'TickLabelInterpreter','latex');
view([0 0])