function [ theta1,theta2,theta3,theta4,theta5,theta6 ] = Func_ikine_7bot( A )
%求解逆运动学方程
a1 = 0.035;
a2 = 0.15;
d4 = 0.2206;
d6 = 0.012;
nx=A(1,1);ox=A(1,2);ax=A(1,3);px=A(1,4);
ny=A(2,1);oy=A(2,2);ay=A(2,3);py=A(2,4);
nz=A(3,1);oz=A(3,2);az=A(3,3);pz=A(3,4);

theta1 = atan((ay*d6-py)/(ax*d6-px))

s1 = sin(theta1);
c1 = cos(theta1);
eq1 = px*c1 + py*s1 - a1 - d6*ax*c1 - d6*ay*s1;
eq2 = pz - d6*az;
eq3 = (a2^2 - eq1^2 - d4^2 - eq2^2)/(2*d4);
a = eq1^2 + eq2^2;
b = 2*eq1*eq3;
c = eq3^2 - eq2^2;
s23 = (-b + (b^2 - 4*a*c)^0.5)/(2*a);
c23 = (eq3 + eq1*s23)/eq2;
theta23 = atan2(s23,c23);
s2 = (eq2 + d4*c23)/a2;
c2 = (eq1 - d4*s23)/a2;
theta2 = atan2(s2,c2)

theta3 = theta23 - theta2

theta4 = atan((ax*s1 - ay*c1)/(az*s23 + ax*c1*c23 + ay*s1*c23))

s4 = sin(theta4);
c5 = -az*c23 + ax*c1*s23 + ay*s1*s23;
s5 = (ax*s1 - ay*c1)/s4;
theta5=atan2(s5,c5)

theta6=atan(-(-oz*c23+ox*c1*s23+oy*s1*s23)/(-nz*c23+nx*c1*s23+ny*s1*s23))

end

