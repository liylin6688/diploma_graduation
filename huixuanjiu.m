clc
clear
%% 机械臂建模

L1 = Link( [ 0    0         0.035    pi/2   0], 'standard' ) ;
L2 = Link( [ 0    0         0.15     0      0], 'standard' ) ;
L3 = Link( [ 0    0         0        pi/2   0], 'standard' ) ;
L4 = Link( [ 0    0.2206    0        -pi/2  0], 'standard' ) ;
L5 = Link( [ 0    0         0        pi/2   0], 'standard' ) ;
L6 = Link( [ 0    0.012     0        0      0], 'standard' ) ;
bot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', '7bot');%连接连杆 

%% 求解正运动学方程
InitDEG = [90 115 -25 0 70 90]%初始角度

a1 = 0.035;
a2 = 0.15;
d4 = 0.2206;
d6 = 0.012;

TRANSM = Func_fkine(InitDEG,a1,a2,d4,d6)
 %% 回旋灸轨迹生成   
 TRANSMM = TRANSM; 
 R = 0.05;
 N = 40;
 j = 1; 
 for deg = 0:2*pi/N:2*pi
        TM(:,:,j) = TRANSMM*[R*cos(deg),R*sin(deg),0,1]';
        TRANSM(1,4) = TM(1,1,j);
        TRANSM(2,4) = TM(2,1,j);
        TRANSM(3,4) = TM(3,1,j);
        
        FinalDEG = Func_ikine(TRANSM,InitDEG);
        InitDEG = FinalDEG;
        radrad(j,:) = InitDEG.*pi/180;
         j = j+1;
 end
 %% 回旋灸显示
 for ii = 1:50
    bot.plot(radrad)
 end
%% 画出轨迹曲线
% x=squeeze(TM(1,1,:));
% y=squeeze(TM(2,1,:));
% z=squeeze(TM(3,1,:));
% plot3(x,y,z);xlabel('X/m');ylabel('Y/m');zlabel('Z/m');title('回旋灸轨迹图');grid on


