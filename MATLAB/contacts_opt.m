%% 2D case: x=[x1,y1,...,x4,y4,Fz1,...,Fz4];
clear all
close all
clc

x0=zeros(12,1); 

Fd=100;

fun=@(x)[ x(9)+x(10)+x(11)+x(12) - Fd; x(2)*x(9)+x(4)*x(10)+x(6)*x(11)+x(8)*x(12);-x(1)*x(9)-x(3)*x(10)-x(5)*x(11)-x(7)*x(12) ]'* ...
        [ x(9)+x(10)+x(11)+x(12) - Fd; x(2)*x(9)+x(4)*x(10)+x(6)*x(11)+x(8)*x(12);-x(1)*x(9)-x(3)*x(10)-x(5)*x(11)-x(7)*x(12) ];

ub=[2;-.2;2;.8;-1;-.2;-1;.8;100;100;100;100];
lb=[1;-.8;1;.2;-2;-.8;-2;.2;0;0;0;0];
      
x = fmincon(fun,x0,[],[],[],[],lb,ub);

F=x(9:12)

figure
plot(x(1),x(2),'r*');
hold on
plot(x(3),x(4),'r*');
hold on
plot(x(5),x(6),'r*');
hold on
plot(x(7),x(8),'r*');
hold on;
fill([1 2 2 1],[-.8 -.8 -.2 -.2],[.8 .8 .8]);alpha(0.2);
hold on;
fill([1 2 2 1],[.2 .2 .8 .8],[.8 .8 .8]);alpha(0.2);
hold on;
fill([-1 -2 -2 -1],[-.8 -.8 -.2 -.2],[.8 .8 .8]);alpha(0.2);
hold on;
fill([-1 -2 -2 -1],[.2 .2 .8 .8],[.8 .8 .8]);alpha(0.2);
xlim([-2 2]);
ylim([-1 1]);
line([0,0], ylim, 'Color', 'k', 'LineWidth', 1); % Draw line for Y axis.
line(xlim, [0,0], 'Color', 'k', 'LineWidth', 1); % Draw line for X axis.
grid on

%% 3D case: x=[x1,y1,z1,...,x4,y4,z4,Fx1,Fy1,Fz1,...,Fx4,Fy4,Fz4];
clear all
close all
clc

Fd=[0;0;100];
tau_d=[0;0;0];

F_max=100;

fun=@(x)[x(13)+x(16)+x(19)+x(22)-Fd(1);x(14)+x(17)+x(20)+x(23)-Fd(2);x(15)+x(18)+x(21)+x(24)-Fd(3);...
        -x(3)*x(14)-x(2)*x(15)-x(6)*x(17)-x(5)*x(18)-x(9)*x(20)-x(8)*x(21)-x(12)*x(23)-x(11)*x(24)-tau_d(1);...
         x(3)*x(13)-x(1)*x(15)+x(6)*x(16)-x(4)*x(18)+x(9)*x(19)-x(7)*x(21)+x(12)*x(22)-x(10)*x(24)-tau_d(2);...
        -x(2)*x(13)+x(1)*x(14)-x(5)*x(16)+x(4)*x(17)-x(8)*x(19)+x(7)*x(20)-x(11)*x(22)+x(10)*x(23)-tau_d(3)]'*...
        ...
        [x(13)+x(16)+x(19)+x(22)-Fd(1);x(14)+x(17)+x(20)+x(23)-Fd(2);x(15)+x(18)+x(21)+x(24)-Fd(3);...
        -x(3)*x(14)-x(2)*x(15)-x(6)*x(17)-x(5)*x(18)-x(9)*x(20)-x(8)*x(21)-x(12)*x(23)-x(11)*x(24)-tau_d(1);...
         x(3)*x(13)-x(1)*x(15)+x(6)*x(16)-x(4)*x(18)+x(9)*x(19)-x(7)*x(21)+x(12)*x(22)-x(10)*x(24)-tau_d(2);...
        -x(2)*x(13)+x(1)*x(14)-x(5)*x(16)+x(4)*x(17)-x(8)*x(19)+x(7)*x(20)-x(11)*x(22)+x(10)*x(23)-tau_d(3)];

x0=zeros(24,1); 

% friction cones
mu=sqrt(2)/2 * .2;

A=[zeros(1,12), 1,0,-mu,zeros(1,9);
   zeros(1,12),-1,0,-mu,zeros(1,9);
   zeros(1,15), 1,0,-mu,zeros(1,6);
   zeros(1,15),-1,0,-mu,zeros(1,6);
   zeros(1,18), 1,0,-mu,zeros(1,3);
   zeros(1,18),-1,0,-mu,zeros(1,3);
   zeros(1,21), 1,0,-mu;
   zeros(1,21),-1,0,-mu;
   zeros(1,12),0, 1,-mu,zeros(1,9);
   zeros(1,12),0,-1,-mu,zeros(1,9);
   zeros(1,15),0, 1,-mu,zeros(1,6);
   zeros(1,15),0,-1,-mu,zeros(1,6);
   zeros(1,18),0, 1,-mu,zeros(1,3);
   zeros(1,18),0,-1,-mu,zeros(1,3);
   zeros(1,21),0, 1,-mu;
   zeros(1,21),0,-1,-mu;
   zeros(1,12),0,0, 1,zeros(1,9);
   zeros(1,12),0,0,-1,zeros(1,9);
   zeros(1,15),0,0, 1,zeros(1,6);
   zeros(1,15),0,0,-1,zeros(1,6);
   zeros(1,18),0,0, 1,zeros(1,3);
   zeros(1,18),0,0,-1,zeros(1,3);
   zeros(1,21),0,0, 1,
   zeros(1,21),0,0,-1,];  

b=[zeros(16,1);F_max;0;F_max;0;F_max;0;F_max;0];
 
% rectangular contact surfaces on the (x,y) plane
ub=[2;-.2;0;2;.8;0;-1;-.2;0;-1;.8;0; F_max*ones(12,1)];
lb=[1;-.8;0;1;.2;0;-2;-.8;0;-2;.2;0;-F_max*ones(12,1)];
      
x = fmincon(fun,x0,A,b,[],[],lb,ub);

F=[x(13:15),x(16:18),x(19:21),x(22:24)]
p=[x(1:3),x(4:6),x(7:9),x(10:12)]
      
figure
plot(x(1),x(2),'r*');
hold on
plot(x(4),x(5),'r*');
hold on
plot(x(7),x(8),'r*');
hold on
plot(x(10),x(11),'r*');
hold on;
fill([1 2 2 1],[-.8 -.8 -.2 -.2],[.8 .8 .8]);alpha(0.2);
hold on;
fill([1 2 2 1],[.2 .2 .8 .8],[.8 .8 .8]);alpha(0.2);
hold on;
fill([-1 -2 -2 -1],[-.8 -.8 -.2 -.2],[.8 .8 .8]);alpha(0.2);
hold on;
fill([-1 -2 -2 -1],[.2 .2 .8 .8],[.8 .8 .8]);alpha(0.2);
xlim([-2 2]);
ylim([-1 1]);
% line([0,0], ylim, 'Color', 'k', 'LineWidth', .5); % Draw line for Y axis.
% line(xlim, [0,0], 'Color', 'k', 'LineWidth', .5); % Draw line for X axis.
xlabel('$x$ axis','Interpreter','latex');
ylabel('$y$ axis','Interpreter','latex');
set(gca,'TickLabelInterpreter','latex');
% set(gca,'FontSize',12);
grid on

%% COST FUNCTION
clear all
close all
clc

Fd=[20;0;100];
tau_d=[0;0;0];

F_max=100;

fun=@(x)[x(13)+x(16)+x(19)+x(22)-Fd(1);x(14)+x(17)+x(20)+x(23)-Fd(2);x(15)+x(18)+x(21)+x(24)-Fd(3);...
        -x(3)*x(14)-x(2)*x(15)-x(6)*x(17)-x(5)*x(18)-x(9)*x(20)-x(8)*x(21)-x(12)*x(23)-x(11)*x(24)-tau_d(1);...
         x(3)*x(13)-x(1)*x(15)+x(6)*x(16)-x(4)*x(18)+x(9)*x(19)-x(7)*x(21)+x(12)*x(22)-x(10)*x(24)-tau_d(2);...
        -x(2)*x(13)+x(1)*x(14)-x(5)*x(16)+x(4)*x(17)-x(8)*x(19)+x(7)*x(20)-x(11)*x(22)+x(10)*x(23)-tau_d(3)]'*...
        ...
        [x(13)+x(16)+x(19)+x(22)-Fd(1);x(14)+x(17)+x(20)+x(23)-Fd(2);x(15)+x(18)+x(21)+x(24)-Fd(3);...
        -x(3)*x(14)-x(2)*x(15)-x(6)*x(17)-x(5)*x(18)-x(9)*x(20)-x(8)*x(21)-x(12)*x(23)-x(11)*x(24)-tau_d(1);...
         x(3)*x(13)-x(1)*x(15)+x(6)*x(16)-x(4)*x(18)+x(9)*x(19)-x(7)*x(21)+x(12)*x(22)-x(10)*x(24)-tau_d(2);...
        -x(2)*x(13)+x(1)*x(14)-x(5)*x(16)+x(4)*x(17)-x(8)*x(19)+x(7)*x(20)-x(11)*x(22)+x(10)*x(23)-tau_d(3)];

x0=randn(24,1); 

% friction cones
mu=sqrt(2)/2 * .2;

A=[zeros(1,12), 1,0,-mu,zeros(1,9);
   zeros(1,12),-1,0,-mu,zeros(1,9);
   zeros(1,15), 1,0,-mu,zeros(1,6);
   zeros(1,15),-1,0,-mu,zeros(1,6);
   zeros(1,18), 1,0,-mu,zeros(1,3);
   zeros(1,18),-1,0,-mu,zeros(1,3);
   zeros(1,21), 1,0,-mu;
   zeros(1,21),-1,0,-mu;
   zeros(1,12),0, 1,-mu,zeros(1,9);
   zeros(1,12),0,-1,-mu,zeros(1,9);
   zeros(1,15),0, 1,-mu,zeros(1,6);
   zeros(1,15),0,-1,-mu,zeros(1,6);
   zeros(1,18),0, 1,-mu,zeros(1,3);
   zeros(1,18),0,-1,-mu,zeros(1,3);
   zeros(1,21),0, 1,-mu;
   zeros(1,21),0,-1,-mu;
   zeros(1,12),0,0, 1,zeros(1,9);
   zeros(1,12),0,0,-1,zeros(1,9);
   zeros(1,15),0,0, 1,zeros(1,6);
   zeros(1,15),0,0,-1,zeros(1,6);
   zeros(1,18),0,0, 1,zeros(1,3);
   zeros(1,18),0,0,-1,zeros(1,3);
   zeros(1,21),0,0, 1,
   zeros(1,21),0,0,-1,];  

R=eye(24,24);
% Rotation matrix S4
R(22:24,22:24)=[0 0 -1; 0 1 0; 1 0 0];
A=A*R;

b=[zeros(16,1);F_max;0;F_max;0;F_max;0;F_max;0];
 
% rectangular contact surfaces: S1,S2,S3 (x,y) plane - S4 (y,z) plane  
ub=[2;-.2;0;2;.8;0;-1;-.2;0;-1;.8;.4; F_max*ones(12,1)];
lb=[1;-.8;0;1;.2;0;-2;-.8;0;-1;.2;.2;-F_max*ones(12,1)];

options = optimoptions('fmincon','OptimalityTolerance',1e-4,'MaxFunctionEvaluations',6000);
      
[x,fval,exitflag,output,lambda,grad,hessian] = fmincon(fun,x0,A,b,[],[],lb,ub,[],options);

F=[x(13:15),x(16:18),x(19:21),x(22:24)]
p=[x(1:3),x(4:6),x(7:9),x(10:12)]
      
fig=figure;
fig.Units='centimeters';
fig.Position=[10 5 20 22];
subplot(2,1,1)
plot(x(1),x(2),'r*');
hold on
plot(x(4),x(5),'r*');
hold on
plot(x(7),x(8),'r*');
hold on
plot(x(10),x(11),'r*');
hold on;
fill([1 2 2 1],[-.8 -.8 -.2 -.2],[.8 .8 .8]);alpha(0.2);
hold on;
fill([1 2 2 1],[.2 .2 .8 .8],[.8 .8 .8]);alpha(0.2);
hold on;
fill([-1 -2 -2 -1],[-.8 -.8 -.2 -.2],[.8 .8 .8]);alpha(0.2);
hold on;
fill([-1 -1 -1 -1],[.2 .2 .8 .8],[.8 .8 .8]);alpha(0.2);
xlim([-2 2]);
ylim([-1 1]);
% line([0,0], ylim, 'Color', 'k', 'LineWidth', 1); % Draw line for Y axis.
% line(xlim, [0,0], 'Color', 'k', 'LineWidth', 1); % Draw line for X axis.
xlabel('$x$ axis','Interpreter','latex');
ylabel('$y$ axis','Interpreter','latex');
set(gca,'TickLabelInterpreter','latex');
% set(gca,'FontSize',12);
grid on

subplot(2,1,2)
plot(x(2),x(3),'r*');
hold on
plot(x(5),x(6),'r*');
hold on
plot(x(8),x(9),'r*');
hold on
plot(x(11),x(12),'r*');
hold on;
fill([.2 .8 .8 .2],[.2 .2 .4 .4],[.8 .8 .8]);alpha(0.2);
xlim([-1 1]);
ylim([0 1]);
% line([0,0], ylim, 'Color', 'k', 'LineWidth', 1); % Draw line for Y axis.
% line(xlim, [0,0], 'Color', 'k', 'LineWidth', 1); % Draw line for X axis.
xlabel('$y$ axis','Interpreter','latex');
ylabel('$z$ axis','Interpreter','latex');
set(gca,'TickLabelInterpreter','latex');
% set(gca,'FontSize',12);
grid on

%% CONSTRAINT formulation
clear all
close all
clc

Fd=[20;0;100];
tau_d=[0;0;0];

F_max=100;

fun=@(x) x(13:24)'* x(13:24);

x0=randn(24,1); 

% friction cones
coeff=0.2;
mu=sqrt(2*coeff)/2;

A=[zeros(1,12), 1,0,-mu,zeros(1,9);
   zeros(1,12),-1,0,-mu,zeros(1,9);
   zeros(1,15), 1,0,-mu,zeros(1,6);
   zeros(1,15),-1,0,-mu,zeros(1,6);
   zeros(1,18), 1,0,-mu,zeros(1,3);
   zeros(1,18),-1,0,-mu,zeros(1,3);
   zeros(1,21), 1,0,-mu;
   zeros(1,21),-1,0,-mu;
   zeros(1,12),0, 1,-mu,zeros(1,9);
   zeros(1,12),0,-1,-mu,zeros(1,9);
   zeros(1,15),0, 1,-mu,zeros(1,6);
   zeros(1,15),0,-1,-mu,zeros(1,6);
   zeros(1,18),0, 1,-mu,zeros(1,3);
   zeros(1,18),0,-1,-mu,zeros(1,3);
   zeros(1,21),0, 1,-mu;
   zeros(1,21),0,-1,-mu;
   zeros(1,12),0,0, 1,zeros(1,9);
   zeros(1,12),0,0,-1,zeros(1,9);
   zeros(1,15),0,0, 1,zeros(1,6);
   zeros(1,15),0,0,-1,zeros(1,6);
   zeros(1,18),0,0, 1,zeros(1,3);
   zeros(1,18),0,0,-1,zeros(1,3);
   zeros(1,21),0,0, 1;
   zeros(1,21),0,0,-1];  

R=eye(24,24);
% Rotation matrix S4
R(22:24,22:24)=[0 0 -1; 0 1 0; 1 0 0];
A=A*R;

b=[zeros(16,1);F_max;0;F_max;0;F_max;0;F_max;0];
 
% rectangular contact surfaces: S1,S2,S3 (x,y) plane - S4 (y,z) plane  
ub=[2;-.2;0;2;.8;0;-1;-.2;0;-1;.8;.4; F_max*ones(12,1)];
lb=[1;-.8;0;1;.2;0;-2;-.8;0;-1;.2;.2;-F_max*ones(12,1)];
      
[x,fval,exitflag,output,lambda,grad,hessian] = fmincon(fun,x0,A,b,[],[],lb,ub,@mycon);

F=[x(13:15),x(16:18),x(19:21),x(22:24)]
p=[x(1:3),x(4:6),x(7:9),x(10:12)]
      
fig=figure;
fig.Units='centimeters';
fig.Position=[10 5 20 22];
subplot(2,1,1)
plot(0,0,'o','MarkerSize',10,'MarkerFaceColor', [0    0.4470    0.7410],'MarkerEdgeColor','k');
hold on;
plot(x(1),x(2),'o','MarkerSize',5,'MarkerFaceColor', 'r','MarkerEdgeColor','k');
hold on
plot(x(4),x(5),'o','MarkerSize',5,'MarkerFaceColor', 'r','MarkerEdgeColor','k');
hold on
plot(x(7),x(8),'o','MarkerSize',5,'MarkerFaceColor', 'r','MarkerEdgeColor','k');
hold on
plot(x(10),x(11),'o','MarkerSize',5,'MarkerFaceColor', 'r','MarkerEdgeColor','k');
hold on;
fill([1 2 2 1],[-.8 -.8 -.2 -.2],[.8 .8 .8]);alpha(0.2);
hold on;
fill([1 2 2 1],[.2 .2 .8 .8],[.8 .8 .8]);alpha(0.2);
hold on;
fill([-1 -2 -2 -1],[-.8 -.8 -.2 -.2],[.8 .8 .8]);alpha(0.2);
hold on;
fill([-1 -1 -1 -1],[.2 .2 .8 .8],[.8 .8 .8]);alpha(0.2);
xlim([-2 2]);
ylim([-1 1]);
xlabel('$x$ axis','Interpreter','latex');
ylabel('$y$ axis','Interpreter','latex');
set(gca,'TickLabelInterpreter','latex');
h=legend('com');
set(h,'Interpreter','latex');set(h,'FontSize',10);
grid on

subplot(2,1,2)
plot(0,0.5,'o','MarkerSize',10,'MarkerFaceColor', [0    0.4470    0.7410],'MarkerEdgeColor','k');
hold on;
plot(x(2),x(3),'o','MarkerSize',5,'MarkerFaceColor', 'r','MarkerEdgeColor','k');
hold on
plot(x(5),x(6),'o','MarkerSize',5,'MarkerFaceColor', 'r','MarkerEdgeColor','k');
hold on
plot(x(8),x(9),'o','MarkerSize',5,'MarkerFaceColor', 'r','MarkerEdgeColor','k');
hold on
plot(x(11),x(12),'o','MarkerSize',5,'MarkerFaceColor', 'r','MarkerEdgeColor','k');
hold on;
fill([.2 .8 .8 .2],[.2 .2 .4 .4],[.8 .8 .8]);alpha(0.2);
xlim([-1 1]);
ylim([0 1]);
xlabel('$y$ axis','Interpreter','latex');
ylabel('$z$ axis','Interpreter','latex');
set(gca,'TickLabelInterpreter','latex');
h=legend('com');
set(h,'Interpreter','latex');set(h,'FontSize',10);
grid on

%% CONSTRAINT formulation - superellipsoid
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
       

%%
close all

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


      