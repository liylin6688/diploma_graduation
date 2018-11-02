function [ M ] = Func_inv_robot( A )
syms nx ny nz ox oy oz ax ay az px py pz;
nx= A(1,1);
ox= A(1,2);
ax= A(1,3);
ny= A(2,1);
oy= A(2,2);
ay= A(2,3);
nz= A(3,1);
oz= A(3,2);
az= A(3,3);
px= -A(1,4)*nx-A(2,4)*ny-A(3,4)*nz;
py= -A(1,4)*ox-A(2,4)*oy-A(3,4)*oz;
pz= -A(1,4)*ax-A(2,4)*ay-A(3,4)*az;
M=  [nx ny nz px;
     ox oy oz py;
     ax ay az pz;
     0  0  0  1  ];
end

