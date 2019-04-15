% CONSTRAINT formulation - superellipsoid
clear all
close all
clc

% addpath('cd home/matteo/Downloads');

Fd=[80;0;100];
tau_d=[0;0;0];

F_max=100;

com=[-0.1 0 0.5];

% fun=@(x) x(13:24)'* x(13:24);

xref=[ 0.5, -0.3, 0, ... 
       0.5,  0.3, 0, ...
      -0.5, -0.3, 0, ...
      -0.5,  0.3, 0, ...
       zeros(1,12)]';
   
 W=[100*ones(12,1);ones(12,1)];

fun=@(x) ((x-xref)'*diag(W)*(x-xref));

x0=randn(24,1); 

ub1=[2 -.1 .4]'; lb1=[.1 -1 0]';
ub2=[2 1 .4]'; lb2=[.1 .1 0]';
ub3=[-.1 -.1 .4]'; lb3=[-2 -1 0]';
ub4=[-.1 1 .4]'; lb4=[-2 .1 0]';

ub=[ub1;ub2;ub3;ub4;F_max*ones(12,1)];
lb=[lb1;lb2;lb3;lb4;-F_max*ones(12,1)];

options = optimoptions('fmincon','MaxFunctionEvaluations',6000);
      
constraint_fun = @(x) mycon_super_ellipsoid(x,Fd,tau_d,com,F_max);

[x,fval,exitflag,output,lambda,grad,hessian] = fmincon(fun,x0,[],[],[],[],lb,ub,constraint_fun,options);

F=[x(13:15),x(16:18),x(19:21),x(22:24)]
p=[x(1:3),x(4:6),x(7:9),x(10:12)]
       

% PLOT FMINCON

close all

addpath('superquadratics');

[x_s,y_s,z_s] = sphere;
[x_e, y_e, z_e]=superellipsoid([1.5 0 1],[2 1 1],[8 8 4],100);

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
vectarrow(x(1:3),x(1:3) + .5*x(13:15)/100);
hold on;
vectarrow(x(4:6),x(4:6) + .5*x(16:18)/100);
hold on;
vectarrow(x(7:9),x(7:9) + .5*x(19:21)/100);
hold on;
vectarrow(x(10:12),x(10:12) + .5*x(22:24)/100);
hold on;
vectarrow(com',com' + .5*Fd/100);
grid on;
xlabel('$x$ axis','Interpreter','latex');
ylabel('$y$ axis','Interpreter','latex');
zlabel('$z$ axis','Interpreter','latex');
xlim([-0.6 0.4]);
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
vectarrow(x(1:3),x(1:3) + .5*x(13:15)/100);
hold on;
vectarrow(x(4:6),x(4:6) + .5*x(16:18)/100);
hold on;
vectarrow(x(7:9),x(7:9) + .5*x(19:21)/100);
hold on;
vectarrow(x(10:12),x(10:12) + .5*x(22:24)/100);
hold on;
vectarrow(com',com' + .5*Fd/100);
grid on;
xlabel('$x$ axis','Interpreter','latex');
ylabel('$y$ axis','Interpreter','latex');
zlabel('$z$ axis','Interpreter','latex');
xlim([-0.6 0.4]);
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
vectarrow(x(1:3),x(1:3) + .5*x(13:15)/100);
hold on;
vectarrow(x(4:6),x(4:6) + .5*x(16:18)/100);
hold on;
vectarrow(x(7:9),x(7:9) + .5*x(19:21)/100);
hold on;
vectarrow(x(10:12),x(10:12) + .5*x(22:24)/100);
hold on;
vectarrow(com',com' + .5*Fd/100);
grid on;
xlabel('$x$ axis','Interpreter','latex');
ylabel('$y$ axis','Interpreter','latex');
zlabel('$z$ axis','Interpreter','latex');
xlim([-0.6 0.4]);
ylim([-0.5 0.5]);
zlim([0 1]);
set(gca,'TickLabelInterpreter','latex');
view([0 0])



      