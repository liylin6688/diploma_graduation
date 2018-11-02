clc
clear
%%
%           theta d       a        alpha  sigma
L1 = Link( [ 0    0       0.035   pi/2   0], 'standard' ) ;
L2 = Link( [ 0    0       0.15     0      0], 'standard' ) ;
L3 = Link( [ 0    0       0        pi/2   0], 'standard' ) ;
L4 = Link( [ 0    0.2206  0        -pi/2  0], 'standard' ) ;
L5 = Link( [ 0    0       0        pi/2   0], 'standard' ) ;
L6 = Link( [ 0    0.012   0        0      0], 'standard' ) ;
bot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', '7bot');%�������� 

%% ���������
InitDEG=[90 115 -25 0 -40 90]
% InitDEG = [85 105 -15 90 40 90]
InitRAD=InitDEG.*pi/180;
Init_M = bot.fkine(InitRAD);
initp = [Init_M(1,4) Init_M(2,4) Init_M(3,4)];
%% ���յ�����
L6 = Link( [ 0    0.097   0        0      0], 'standard' ) ;
bot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', '7bot');%�������� 
Final_M = bot.fkine(InitRAD)
finalp = [Final_M(1,4) Final_M(2,4) Final_M(3,4)];
%% ��ȸ�ľĹ켣����
Trans_M = Init_M;
j = 1; 
for i = 0:0.01:0.06
    Trans_M(1,4) = i/0.15*(Final_M(1,4) - Init_M(1,4)) + Init_M(1,4);
    Trans_M(2,4) = i/0.15*(Final_M(2,4) - Init_M(2,4)) + Init_M(2,4);
    Trans_M(3,4) = i/0.15*(Final_M(3,4) - Init_M(3,4)) + Init_M(3,4);
    
    FinalDEG = Func_ikine(Trans_M,InitDEG);
    InitDEG = FinalDEG;
    radrad(j,:) = InitDEG.*pi/180;
    j = j+1;
end
%% ȸ�ľĶ�������
k=0;dirr = 1;
for a = 1:1000
    if k == 1
        dirr = 1;
    end
    if k == (j - 1)
        dirr = 0;
    end
    if dirr == 1
        k = k + 1;
    end
    if dirr == 0
        k = k - 1;
    end
    bot.plot(radrad(k,:))
end




