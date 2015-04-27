%% 将 航向角 转换到 -180~180 

function yaw_Out = yawHandle(yaw_In)

N = size(yaw_In,2);
yaw_Out = zeros(1,N);
for k=1:N
    yaw_In_k = yaw_In(k);
    yaw_Out(k)=yaw_In_k;
    if yaw_In_k>pi
        yaw_Out(k) = yaw_In_k-2*pi ;
    end
    if yaw_In_k<-pi
        yaw_Out(k) = yaw_In_k+2*pi ;
    end
end
