function [c,ceq] = mycon_super_ellipsoid(x,Fd,tau_d,com,F_max)


% super-ellipsoid
p=[8 8 4]; % r s t parameters (r=s super-ellipsoid)
r=[2 5 1]; % axis radius
C=[1.5 0 1]; % center 

% Compute nonlinear inequalities at x.

% friction cones
coeff=.2;
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

% normal vectors

n1 = -[p(1)/(r(1)^p(1))*(x(1)-C(1))^(p(1)-1);  p(2)/(r(2)^p(2))*(x(2)-C(2))^(p(2)-1);  p(3)/(r(3)^p(3))*(x(3)-C(3))^(p(3)-1)];
n2 = -[p(1)/(r(1)^p(1))*(x(4)-C(1))^(p(1)-1);  p(2)/(r(2)^p(2))*(x(5)-C(2))^(p(2)-1);  p(3)/(r(3)^p(3))*(x(6)-C(3))^(p(3)-1)];
n3 = -[p(1)/(r(1)^p(1))*(x(7)-C(1))^(p(1)-1);  p(2)/(r(2)^p(2))*(x(8)-C(2))^(p(2)-1);  p(3)/(r(3)^p(3))*(x(9)-C(3))^(p(3)-1)];
n4 = -[p(1)/(r(1)^p(1))*(x(10)-C(1))^(p(1)-1); p(2)/(r(2)^p(2))*(x(11)-C(2))^(p(2)-1); p(3)/(r(3)^p(3))*(x(12)-C(3))^(p(3)-1)];

n1= n1/norm(n1);
n2= n2/norm(n2);
n3= n3/norm(n3);
n4= n4/norm(n4);

R1 = [ n1(2)/norm(n1(1:2)), -n1(1)/norm(n1(1:2)), 0;
      (n1(1)*n1(3))/norm(n1(1:2)), (n1(2)*n1(3))/norm(n1(1:2)), -norm(n1(1:2));
       n1(1) n1(2) n1(3)];

R2 = [ n2(2)/norm(n2(1:2)), -n2(1)/norm(n2(1:2)), 0;
      (n2(1)*n2(3))/norm(n2(1:2)), (n2(2)*n2(3))/norm(n2(1:2)), -norm(n2(1:2));
       n2(1) n2(2) n2(3)];

R3 = [ n3(2)/norm(n3(1:2)), -n3(1)/norm(n3(1:2)), 0;
      (n3(1)*n3(3))/norm(n3(1:2)), (n3(2)*n3(3))/norm(n3(1:2)), -norm(n3(1:2));
       n3(1) n3(2) n3(3)];

R4 = [ n4(2)/norm(n4(1:2)), -n4(1)/norm(n4(1:2)), 0;
      (n4(1)*n4(3))/norm(n4(1:2)), (n4(2)*n4(3))/norm(n4(1:2)), -norm(n4(1:2));
       n4(1) n4(2) n4(3)];

R=eye(24,24);
R(13:15,13:15)=R1;
R(16:18,16:18)=R2;
R(19:21,19:21)=R3;
R(22:24,22:24)=R4;

A=A*R;

b=[zeros(16,1);F_max;0;F_max;0;F_max;0;F_max;0];

c = A*x-b; 

%  c=[];

% Compute nonlinear equalities at x.

S1=[0,-(x(3)-com(3)),x(2)-com(2);x(3)-com(3),0,-(x(1)-com(1));-(x(2)-com(2)),x(1)-com(1),0]; 
S2=[0,-(x(6)-com(3)),x(5)-com(2);x(6)-com(3),0,-(x(4)-com(1));-(x(5)-com(2)),x(4)-com(1),0];
S3=[0,-(x(9)-com(3)),x(8)-com(2);x(9)-com(3),0,-(x(7)-com(1));-(x(8)-com(2)),x(7)-com(1),0];
S4=[0,-(x(12)-com(3)),x(11)-com(2);x(12)-com(3),0,-(x(10)-com(1));-(x(11)-com(2)),x(10)-com(1),0];

% super-ellipsoid

SE1 = (x(1)-C(1))^p(1)/(r(1)^p(1))  + (x(2)-C(2))^p(2)/(r(2)^p(2))  + (x(3)-C(3))^p(3)/(r(3)^p(3))  -1;
SE2 = (x(4)-C(1))^p(1)/(r(1)^p(1))  + (x(5)-C(2))^p(2)/(r(2)^p(2))  + (x(6)-C(3))^p(3)/(r(3)^p(3))  -1;
SE3 = (x(7)-C(1))^p(1)/(r(1)^p(1))  + (x(8)-C(2))^p(2)/(r(2)^p(2))  + (x(9)-C(3))^p(3)/(r(3)^p(3))  -1;
SE4 = (x(10)-C(1))^p(1)/(r(1)^p(1)) + (x(11)-C(2))^p(2)/(r(2)^p(2)) + (x(12)-C(3))^p(3)/(r(3)^p(3)) -1;


G=[zeros(3,12),eye(3),eye(3),eye(3),eye(3); 
   zeros(3,12),S1,S2,S3,S4];

ceq=[G*x-[Fd;tau_d];SE1;SE2;SE3;SE4];

end

