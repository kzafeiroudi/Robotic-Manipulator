clear all
close all
clc

syms c1
syms c2
syms c3
syms c4
syms s1
syms s2
syms s3
syms s4
syms l0
syms l1
syms l2
syms l3
syms l4
syms l5
a01 = [1 0 0 l1; 0 0 -1 0; 0 1 0 -l0; 0 0 0 1;];
a12 = [s1 0 c1 l3*s1; -c1 0 s1 -l3*c1; 0 -1 0 0; 0 0 0 1;];
a23 = [c2 -s2 0 l4*c2; s2 c2 0 l4*s2; 0 0 1 l2; 0 0 0 1;];
a34 = [c3 -s3 0 l5*c3; s3 c3 0 l5*s3; 0 0 1 0; 0 0 0 1;];
a02 = a01*a12;
a03 = a01*a12*a23;
a04 = a03*a34;

syms q1 q2 q3
s1 = sin(q1);
s2 = sin(q2);
s3 = sin(q3);
c1 = cos(q1);
c2 = cos(q2);
c3 = cos(q3);
a01 = [1 0 0 l1; 0 0 -1 0; 0 1 0 -l0; 0 0 0 1;];
a12 = [sin(q1) 0 cos(q1) l3*sin(q1); -cos(q1) 0 sin(q1) -l3*cos(q1); 0 -1 0 0; 0 0 0 1;];
a23 = [cos(q2) -sin(q2) 0 l4*cos(q2); sin(q2) cos(q2) 0 l4*sin(q2); 0 0 1 l2; 0 0 0 1;];
a34 = [cos(q3) -sin(q3) 0 l5*cos(q3); sin(q3) cos(q3) 0 l5*sin(q3); 0 0 1 0; 0 0 0 1;];
a02 = a01*a12;
a03 = a01*a12*a23;
a04 = a03*a34;
q = [q1 q2 q3];
p = a04(1:3,4);
jL = jacobian(p,q);
jA = [a01(1:3,3) a02(1:3,3) a03(1:3,3)];
J = [jL ; jA];
D1 = det(jL);
simplify(D1);
D2 = det(jA);
I = inv(jL);

