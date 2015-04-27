
function Oiga_nbb = GetdCnbdtK(Wrbb)
%% 由角速率计算姿态转移矩阵变化率因子
Oiga_nbb = [        0       -Wrbb(3)    Wrbb(2)
            Wrbb(3)         0       -Wrbb(1)
            -Wrbb(2)    Wrbb(1)        0   ];