clear all
close all
clc

n = sym('n', [3,1], 'real');
assume(n, 'real')
F = sym('F', [3,1], 'real');
assume(F, 'real')

value1 = F-(dot(n, F))*n;
value2 = (dot(value1,value1))^.5;

J1= diff(value2,'F1');
J2= diff(value2,'F2');
J3= diff(value2,'F3');
J4= diff(value2,'n1');
J5= diff(value2,'n2');
J6= diff(value2,'n3');
ccode(J1,'File','J1.c')
ccode(J2,'File','J2.c')
ccode(J3,'File','J3.c')
ccode(J4,'File','J4.c')
ccode(J5,'File','J5.c')
ccode(J6,'File','J6.c')
