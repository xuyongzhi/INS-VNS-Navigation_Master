%%  计算位置误差指标
function errorStr = CalPosErrorIndex( true_position,PosError,AttitudeError )

%% 计算空间二维位置误差及相对值
routeLong_2dim = 0; % 平面总行程
for k=1:length(true_position)-1
    routeLong_2dim = routeLong_2dim + sqrt( (true_position(1,k+1)-true_position(1,k))^2 + (true_position(2,k+1)-true_position(2,k))^2 );
end
% 计算终点误差
PosErrorLength = length(PosError);
endAbsError_2dim = sqrt( (PosError(1,PosErrorLength))^2+(PosError(2,PosErrorLength))^2 ) ;  % 平面终点误差绝对值
endRelError_2dim = endAbsError_2dim/routeLong_2dim ;    % 平面终点误差相对值
% 计算最大误差
maxAbsError_2dim = 0;   % 平面最大误差绝对值
for k=1:PosErrorLength
    error = sqrt( (PosError(1,k))^2+(PosError(2,k))^2 ) ;
    if error>maxAbsError_2dim
       maxAbsError_2dim = error ; 
    end
end
maxRelError_2dim = maxAbsError_2dim/routeLong_2dim ;    % 平面最大误差相对值

errorStr = sprintf('\t平面：\t总行程：%0.4g m\t\t最大误差：%0.4g m (%0.4g%%)\n',...
    routeLong_2dim,maxAbsError_2dim,maxRelError_2dim*100);

%% 计算空间三维位置误差及相对值
routeLong_3dim = 0; % 空间总行程
for k=1:length(true_position)-1
    routeLong_3dim = routeLong_3dim + sqrt( (true_position(1,k+1)-true_position(1,k))^2 + (true_position(2,k+1)-true_position(2,k))^2 + (true_position(3,k+1)-true_position(3,k))^2 );
end
% 计算终点误差
PosErrorLength = length(PosError);
endAbsError_3dim = sqrt( (PosError(1,PosErrorLength))^2+(PosError(2,PosErrorLength))^2+(PosError(3,PosErrorLength))^2 ) ;  % 空间终点误差绝对值
endRelError_3dim = endAbsError_3dim/routeLong_3dim ;    % 空间终点误差相对值
% 计算最大误差
maxAbsError_3dim = 0;   % 空间最大误差绝对值
for k=1:PosErrorLength
    error = sqrt( (PosError(1,k))^2+(PosError(2,k))^2+(PosError(3,k))^2 ) ;
    if error>maxAbsError_3dim
       maxAbsError_3dim = error ; 
    end
end
maxRelError_3dim = maxAbsError_3dim/routeLong_3dim ;    % 空间最大误差相对值

errorStr = sprintf('%s\t空间：\t总行程：%0.4g m\t\t最大误差：%0.4g m (%0.4g%%)\n',errorStr,...
    routeLong_3dim,maxAbsError_3dim,maxRelError_3dim*100);

% 各维的误差
xyzLength = CalRouteLength( true_position ) ; % 得到各维的行程
maxPosError_x = 0;
maxPosError_y = 0;
maxPosError_z = 0;
for k=1:PosErrorLength
    if abs(PosError(1,k))>abs(maxPosError_x)
       maxPosError_x =  PosError(1,k) ;
    end
    if abs(PosError(2,k))>abs(maxPosError_y)
       maxPosError_y =  PosError(2,k) ;
    end
    if abs(PosError(3,k))>abs(maxPosError_z)
       maxPosError_z =  PosError(3,k) ;
    end
end
errorStr = sprintf('%s\t位置误差(x、y、z)：(%0.3g,%0.3g,%0.3g)m\t(%0.3g%%,%0.3g%%,%0.3g%%)\n',...
                errorStr,maxPosError_x,maxPosError_y,maxPosError_z,maxPosError_x/xyzLength(1)*100,maxPosError_y/xyzLength(2)*100,maxPosError_z/xyzLength(3)*100);
% 姿态各位最大误差
if exist('AttitudeError','var')
    maxAtdError_x = 0;
    maxAtdError_y = 0;
    maxAtdError_z = 0;
    for k=1:PosErrorLength
        if abs(AttitudeError(1,k))>abs(maxAtdError_x)
           maxAtdError_x =  AttitudeError(1,k) ;
        end
        if abs(AttitudeError(2,k))>abs(maxAtdError_y)
           maxAtdError_y =  AttitudeError(2,k) ;
        end
        if abs(AttitudeError(3,k))>abs(maxAtdError_z)
           maxAtdError_z =  AttitudeError(3,k) ;
        end
    end
    errorStr = sprintf('%s\t姿态误差 (俯仰、横滚、航向):(%0.3g,%0.3g,%0.3g)〞\n',...
        errorStr,maxAtdError_x,maxAtdError_y,maxAtdError_z);
end
