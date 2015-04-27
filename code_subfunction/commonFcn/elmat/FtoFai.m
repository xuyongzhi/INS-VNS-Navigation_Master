% buaa xyz 2014.1.16

% 状态矩阵F->状态转移矩阵Fai
% cycleT为滤波周期

function Fai = FtoFai(F,cycleT)
format long
step = 1;
Fai = eye(size(F));
for i = 1:10
    step = step*i;
    Fai = Fai + (F * cycleT)^i/step;
end