%% 比较实验IMU和微分反推的IMU

function contrastIMU(trueTrace,imuInputData)

if ~exist('trueTrace','var')
   trueTrace = importdata('trueTrace.mat'); 
   imuInputData = importdata('imuInputData.mat');
   if ~isfield(trueTrace,'dif_wrbb')
    [trueTrace,dif_wrbb,dif_arbr] = get_trueIMU_wr(trueTrace) ;
   end
end

dif_w = trueTrace.dif_wrbb ;
dif_a = trueTrace.dif_arbr ;

a = imuInputData.f ;
w = imuInputData.wib ;

figure
plot([a(1,:);dif_a(1,:)]')
legend('exp','dif')
title('ax')

figure
plot([a(2,:);dif_a(2,:)]')
legend('exp','dif')
title('ay')

figure
plot([a(3,:);dif_a(3,:)]')
legend('exp','dif')
title('az')

figure
plot([w(1,:);dif_w(1,:)]')
legend('exp','dif')
title('wx')

figure
plot([w(2,:);dif_w(2,:)]')
legend('exp','dif')
title('wy')

figure
plot([w(3,:);dif_w(3,:)]')
legend('exp','dif')
title('wz')