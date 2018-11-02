function [ theta1,theta2,theta3,theta4,theta5,theta6 ] = Func_ikine_7bot( A )
%逆运动学方程求得8组解
a1 = 0.035; a2 = 0.15; d4 = 0.2206; d6 = 0.012;

nx = A(1,1); ox = A(1,2); ax = A(1,3); px = A(1,4);
ny = A(2,1); oy = A(2,2); ay = A(2,3); py = A(2,4);
nz = A(3,1); oz = A(3,2); az = A(3,3); pz = A(3,4);

%%

theta1(1:4) = atan((ay*d6-py)/(ax*d6-px));

s1 = sin(theta1(1));
c1 = cos(theta1(1));
eq1 = px*c1 + py*s1 - a1 - d6*ax*c1 - d6*ay*s1;
eq2 = pz - d6*az;
eq3 = (a2^2 - eq1^2 - d4^2 - eq2^2)/(2*d4);
a = eq1^2 + eq2^2;
b = 2*eq1*eq3;
c = eq3^2 - eq2^2;
%%
% *%   s23(1:2)*
s23(1:2) = (-b - (b^2 - 4*a*c)^0.5)/(2*a);
c23 = (eq3 + eq1*s23(1))/eq2;
theta23 = atan2(s23(1),c23);
s2 = (eq2 + d4*c23)/a2;
c2 = (eq1 - d4*s23(1))/a2;
theta2(1:2) = atan2(s2,c2);

theta3(1:2) = theta23 - theta2(1);
%%
% |%    theta4(1)|
theta4(1) = atan((ax*s1 - ay*c1)/(az*s23(1) + ax*c1*c23 + ay*s1*c23));

s4 = sin(theta4(1));
c5 = -az*c23 + ax*c1*s23(1) + ay*s1*s23(1);
s5 = (ax*s1 - ay*c1)/s4;
theta5(1) = atan2(s5,c5);

c6 = (-nz*c23 + nx*c1*s23(1) + ny*s1*s23(1))/(-s5);
s6 = (-oz*c23 + ox*c1*s23(1) + oy*s1*s23(1))/s5;
theta6(1) = atan2(s6,c6);

theta4(2) = atan((ax*s1 - ay*c1)/(az*s23(1) + ax*c1*c23 + ay*s1*c23)) + pi;

s4 = sin(theta4(2));
c5 = -az*c23 + ax*c1*s23(1) + ay*s1*s23(1);
s5 = (ax*s1 - ay*c1)/s4;
theta5(2) = atan2(s5,c5);

c6 = (-nz*c23 + nx*c1*s23(1) + ny*s1*s23(1))/(-s5);
s6 = (-oz*c23 + ox*c1*s23(1) + oy*s1*s23(1))/s5;
theta6(2) = atan2(s6,c6);

%%
% _%    s23(3:4)_
s23(3:4) = (-b + (b^2 - 4*a*c)^0.5)/(2*a);
c23 = (eq3 + eq1*s23(3))/eq2;
theta23 = atan2(s23(3),c23);
s2 = (eq2 + d4*c23)/a2;
c2 = (eq1 - d4*s23(3))/a2;
theta2(3:4) = atan2(s2,c2);

theta3(3:4) = theta23 - theta2(3);
%%
% _%    theta4(3)_
theta4(3) = atan((ax*s1 - ay*c1)/(az*s23(3) + ax*c1*c23 + ay*s1*c23));

s4 = sin(theta4(3));
c5 = -az*c23 + ax*c1*s23(3) + ay*s1*s23(3);
s5 = (ax*s1 - ay*c1)/s4;
theta5(3) = atan2(s5,c5);

c6 = (-nz*c23 + nx*c1*s23(3) + ny*s1*s23(3))/(-s5);
s6 = (-oz*c23 + ox*c1*s23(3) + oy*s1*s23(3))/s5;
theta6(3) = atan2(s6,c6);

theta4(4) = atan((ax*s1 - ay*c1)/(az*s23(3) + ax*c1*c23 + ay*s1*c23)) + pi;

s4 = sin(theta4(4));
c5 = -az*c23 + ax*c1*s23(3) + ay*s1*s23(3);
s5 = (ax*s1 - ay*c1)/s4;
theta5(4) = atan2(s5,c5);

c6 = (-nz*c23 + nx*c1*s23(3) + ny*s1*s23(3))/(-s5);
s6 = (-oz*c23 + ox*c1*s23(3) + oy*s1*s23(3))/s5;
theta6(4) = atan2(s6,c6);

%%
% _% theta1(5:8)*_

theta1(5:8) = atan((ay*d6-py)/(ax*d6-px)) + pi;

s1 = sin(theta1(5));
c1 = cos(theta1(5));
eq1 = px*c1 + py*s1 - a1 - d6*ax*c1 - d6*ay*s1;
eq2 = pz - d6*az;
eq3 = (a2^2 - eq1^2 - d4^2 - eq2^2)/(2*d4);
a = eq1^2 + eq2^2;
b = 2*eq1*eq3;
c = eq3^2 - eq2^2;

%%
% _%   s23(5:6)_

s23(5:6) = (-b - (b^2 - 4*a*c)^0.5)/(2*a);
c23 = (eq3 + eq1*s23(5))/eq2;
theta23 = atan2(s23(5),c23);
s2 = (eq2 + d4*c23)/a2;
c2 = (eq1 - d4*s23(5))/a2;
theta2(5:6) = atan2(s2,c2);

theta3(5:6) = theta23 - theta2(5);

%%
% _% theta4(5)_

theta4(5) = atan((ax*s1 - ay*c1)/(az*s23(5) + ax*c1*c23 + ay*s1*c23));

s4 = sin(theta4(5));
c5 = -az*c23 + ax*c1*s23(5) + ay*s1*s23(5);
s5 = (ax*s1 - ay*c1)/s4;
theta5(5) = atan2(s5,c5);

c6 = (-nz*c23 + nx*c1*s23(5) + ny*s1*s23(5))/(-s5);
s6 = (-oz*c23 + ox*c1*s23(5) + oy*s1*s23(5))/s5;
theta6(5) = atan2(s6,c6);

theta4(6) = atan((ax*s1 - ay*c1)/(az*s23(5) + ax*c1*c23 + ay*s1*c23)) + pi;

s4 = sin(theta4(6));
c5 = -az*c23 + ax*c1*s23(5) + ay*s1*s23(5);
s5 = (ax*s1 - ay*c1)/s4;
theta5(6) = atan2(s5,c5);

c6 = (-nz*c23 + nx*c1*s23(5) + ny*s1*s23(5))/(-s5);
s6 = (-oz*c23 + ox*c1*s23(5) + oy*s1*s23(5))/s5;
theta6(6) = atan2(s6,c6);

%%
% *%   s23(7:8)*

s23(7:8) = (-b + (b^2 - 4*a*c)^0.5)/(2*a);
c23 = (eq3 + eq1*s23(7))/eq2;
theta23 = atan2(s23(7),c23);
s2 = (eq2 + d4*c23)/a2;
c2 = (eq1 - d4*s23(7))/a2;
theta2(7:8) = atan2(s2,c2);

theta3(7:8) = theta23 - theta2(7);

%%
% _%   theta4(7)_

theta4(7) = atan((ax*s1 - ay*c1)/(az*s23(7) + ax*c1*c23 + ay*s1*c23));

s4 = sin(theta4(7));
c5 = -az*c23 + ax*c1*s23(7) + ay*s1*s23(7);
s5 = (ax*s1 - ay*c1)/s4;
theta5(7) = atan2(s5,c5);

c6 = (-nz*c23 + nx*c1*s23(7) + ny*s1*s23(7))/(-s5);
s6 = (-oz*c23 + ox*c1*s23(7) + oy*s1*s23(7))/s5;
theta6(7) = atan2(s6,c6);

theta4(8) = atan((ax*s1 - ay*c1)/(az*s23(7) + ax*c1*c23 + ay*s1*c23)) + pi;

s4 = sin(theta4(8));
c5 = -az*c23 + ax*c1*s23(7) + ay*s1*s23(7);
s5 = (ax*s1 - ay*c1)/s4;
theta5(8) = atan2(s5,c5);

c6 = (-nz*c23 + nx*c1*s23(7) + ny*s1*s23(7))/(-s5);
s6 = (-oz*c23 + ox*c1*s23(7) + oy*s1*s23(7))/s5;
theta6(8) = atan2(s6,c6);

end


