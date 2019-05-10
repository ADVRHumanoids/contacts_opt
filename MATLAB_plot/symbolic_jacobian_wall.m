clear all;
close all;
clc;

syms d0 d1 d2
syms q0 q1 q2

x = d0*cos(q0)+d1*cos(q0+q1)+d2*cos(q0+q1+q2);
y = d0*sin(q0)+d1*sin(q0+q1)+d2*sin(q0+q1+q2);

J = sym('J', [2,3], 'real');
assume(J, 'real');

J= simplify([diff(x,q0) diff(x,q1) diff(x,q2);
             diff(y,q0) diff(y,q1) diff(y,q2)]);

ccode(J,'File','J_wall.c')

manip = simplify(det(J*transpose(J))^.5);

J_manip = sym('J_manip', [3,1], 'real');
assume(J_manip, 'real');

J_manip =[ diff(manip,q0); diff(manip,q1); diff(manip,q2)];

ccode(J_manip,'File','J_manip_wall.c')
