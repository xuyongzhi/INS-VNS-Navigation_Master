% buaa xyz 2014.1.10

% 由向量得到 x乘矩阵:
% 向量V叉乘另一随意向量T 等效于矩阵乘法：VxT=V_crossMatrix*T 

function V_crossMatrix = getCrossMarix( V )
format long
V_crossMatrix = [	0       -V(3)   V(2);
                	V(3)    0       -V(1)
                    -V(2)   V(1)    0   ];

