function A = Func_dh( theta,d,a,alpha,i )
disp(['��',num2str(i),'���˶�ѧ���󷽳�Ϊ��'])
A=[ cos(theta)  -sin(theta)*cos(alpha)  sin(theta)*sin(alpha)   a*cos(theta)
    sin(theta)  cos(theta)*cos(alpha)   -cos(theta)*sin(alpha)  a*sin(theta)
    0           sin(alpha)              cos(alpha)              d
    0           0                       0                       1
  ];

end

