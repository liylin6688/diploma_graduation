本代码基于stm32f4建立六自由度机械臂7Bot的模型及控制：
%           theta d       a        alpha  sigma
L1 = Link( [ 0    0       0.0778   pi/2   0], 'standard' ) ;
L2 = Link( [ 0    0       0.15     0      0], 'standard' ) ;
L3 = Link( [ 0    0       0        pi/2   0], 'standard' ) ;
L4 = Link( [ 0    0.2206  0        -pi/2  0], 'standard' ) ;
L5 = Link( [ 0    0       0        pi/2   0], 'standard' ) ;
L6 = Link( [ 0    0.012   0        0      0], 'standard' ) ;
bot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', '7bot');%连接连杆 