function [ realdeg ] = Func_ikine( Init_M ,InitDEG)

%���룺Init_M�����������ľ���    InitDEG������һ�εĹؽڽǶ�ֵ
%�����realdeg�����Ƕ�ֵ

    [a,b,c,d,e,f] = Func_ikine_7bot(Init_M);
    raddeg = [a' b' c' d' e' f'].*180/pi;
    for i = 1:8
        error(i) = sum(abs(raddeg(i,:)-InitDEG));                              
    end
    [x,y] = min(error);
    realdeg = raddeg(y,:);                  %���õ������Ž�

end

