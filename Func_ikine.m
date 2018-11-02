function [ realdeg ] = Func_ikine( Init_M ,InitDEG)

%输入：Init_M——即将逆解的矩阵    InitDEG——上一次的关节角度值
%输出：realdeg——角度值

    [a,b,c,d,e,f] = Func_ikine_7bot(Init_M);
    raddeg = [a' b' c' d' e' f'].*180/pi;
    for i = 1:8
        error(i) = sum(abs(raddeg(i,:)-InitDEG));                              
    end
    [x,y] = min(error);
    realdeg = raddeg(y,:);                  %逆解得到的最优解

end

