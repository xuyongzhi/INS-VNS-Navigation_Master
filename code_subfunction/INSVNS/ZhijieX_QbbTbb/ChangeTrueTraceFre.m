%% 减小数据的频率

function trueTraceNew = ChangeTrueTraceFre(trueTrace,newFre)

[planet,isKnowTrue,initialPosition_e,initialVelocity_r,initialAttitude_r,trueTraeFre,true_position,true_attitude,true_velocity,true_acc_r] = GetFromTrueTrace( trueTrace );
f_true = trueTrace.f_IMU ;
wib_true = trueTrace.wib_IMU ;

N = length(true_position);
newN = fix((N-1)*newFre/trueTraeFre+1) ;
true_position_new = zeros(3,newN) ;
true_attitude_new = zeros(3,newN) ;
true_velocity_new = zeros(3,newN) ;

for k_new=1:newN
    k = (k_new-1)*trueTraeFre/newFre+1 ;
    true_position_new(:,k_new) = true_position(:,k);
    true_attitude_new(:,k_new) = true_attitude(:,k);
    true_velocity_new(:,k_new) = true_velocity(:,k);
end

true_acc_r_new = zeros(3,newN-1) ;
f_true_new = zeros(3,newN-1) ;
wib_true_new = zeros(3,newN-1) ;
for k_new=1:newN-1
    k = (k_new-1)*trueTraeFre/newFre+1 ;
    true_acc_r_new(:,k_new) = true_acc_r(:,k);
    f_true_new(:,k_new) = f_true(:,k);
    wib_true_new(:,k_new) = wib_true(:,k);
end

trueTraceNew = trueTrace ;
trueTraceNew.true_position = true_position_new ;
trueTraceNew.true_attitude = true_attitude_new ;
trueTraceNew.true_velocity = true_velocity_new ;
trueTraceNew.true_acc_r = true_acc_r_new ;
trueTraceNew.f_true = f_true_new ;
trueTraceNew.wib_true = wib_true_new ;

save trueTraceNew trueTraceNew