clc
clear
%% 函数说明
%link([alpha a theta d sigma],'standard') %9.8版本之前用此函数
%Link([theta d a alpha sigma],'standard') %9.8版本之后用此函数
%bot = robot( { L1 L2 L3 L4 L5 L6} ) ; %9.8版本之前用此函数连接杆件
%drivebot(bot) ; %9.8版本之前用此函数
%teach(bot);%9.8版本之后用此函数
%% 机械臂建模
%           theta d       a        alpha  sigma
L1 = Link( [ 0    0       0.035   pi/2   0], 'standard' ) ;
L2 = Link( [ 0    0       0.15     0      0], 'standard' ) ;
L3 = Link( [ 0    0       0        pi/2   0], 'standard' ) ;
L4 = Link( [ 0    0.2206  0        -pi/2  0], 'standard' ) ;
L5 = Link( [ 0    0       0        pi/2   0], 'standard' ) ;
L6 = Link( [ 0    0.012   0        0      0], 'standard' ) ;
bot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', '7bot');%连接连杆 
bot.display()
%% 正运动学
InitDEG = [90 115 -25 0 25 90];
InitRAD = InitDEG.*pi/180;
Init_M = bot.fkine(InitRAD) 
FinalDEG = [110 115 25 10 25 90];
FinalRAD = FinalDEG.*pi/180;
Final_M = bot.fkine(FinalRAD) 
t =[ 0:0.025:3]';
[q,qd,qdd] = jtraj( InitRAD, FinalRAD, t) ; 
%% 动作
% bot.plot(q) 
% teach(bot)
%% 末端执行器位移
process=bot.fkine(q);
x=squeeze(process(1,4,:));
y=squeeze(process(2,4,:));
z=squeeze(process(3,4,:));
subplot(3,1,1);plot(t,x);xlabel('Time/s');ylabel('X/m');title('x方向位姿变化');grid on
subplot(3,1,2);plot(t,y);xlabel('Time/s');ylabel('Y/m');title('y方向位姿变化');grid on
subplot(3,1,3);plot(t,z);xlabel('Time/s');ylabel('Z/m');title('z方向位姿变化');grid on
%% 关节位移曲线 
figure
subplot(3,2,1);plot(t,q(:,1));xlabel('Time');ylabel('位移1');grid on
subplot(3,2,2);plot(t,q(:,2));xlabel('Time');ylabel('位移2');grid on
subplot(3,2,3);plot(t,q(:,3));xlabel('Time');ylabel('位移3');grid on
subplot(3,2,4);plot(t,q(:,4));xlabel('Time');ylabel('位移4');grid on
subplot(3,2,5);plot(t,q(:,5));xlabel('Time');ylabel('位移5');grid on
subplot(3,2,6);plot(t,q(:,6));xlabel('Time');ylabel('位移6');grid on
%% 关节速度曲线 
figure
subplot(3,2,1);plot(t,qd(:,1));xlabel('Time');ylabel('速度1');grid on
subplot(3,2,2);plot(t,qd(:,2));xlabel('Time');ylabel('速度2');grid on
subplot(3,2,3);plot(t,qd(:,3));xlabel('Time');ylabel('速度3');grid on
subplot(3,2,4);plot(t,qd(:,4));xlabel('Time');ylabel('速度4');grid on
subplot(3,2,5);plot(t,qd(:,5));xlabel('Time');ylabel('速度5');grid on
subplot(3,2,6);plot(t,qd(:,6));xlabel('Time');ylabel('速度6');grid on
%% 关节加速度曲线 
figure
subplot(3,2,1);plot(t,qdd(:,1));xlabel('Time');ylabel('加速度1');grid on
subplot(3,2,2);plot(t,qdd(:,2));xlabel('Time');ylabel('加速度2');grid on
subplot(3,2,3);plot(t,qdd(:,3));xlabel('Time');ylabel('加速度3');grid on
subplot(3,2,4);plot(t,qdd(:,4));xlabel('Time');ylabel('加速度4');grid on
subplot(3,2,5);plot(t,qdd(:,5));xlabel('Time');ylabel('加速度5');grid on
subplot(3,2,6);plot(t,qdd(:,6));xlabel('Time');ylabel('加速度6');grid on

