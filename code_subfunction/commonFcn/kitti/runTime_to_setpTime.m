%%  单位：sec
%  runTime(k): 从开始到现在的时间
%  stepTime(k): 从k到k+1时刻的时间
function stepTime = runTime_to_setpTime(runTime)

stepTime = zeros(size(runTime));
for k=1:length(runTime)-1
    stepTime(k) = runTime(k+1)-runTime(k) ;
end
stepTime(length(runTime)) = [];
