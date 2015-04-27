%%  计算位置误差指标
% 6.12改： 综合误差为 计算行程/真实行程
% 最大/终点位置误差，相对路程的相对误差
% 姿态最大和终点误差
function errorStr = CalPosErrorIndex_route( true_position,PosError,AttitudeError,position_nav )

%% 计算空间二维位置误差及相对值
%% 平面行程误差
routeLong_2dim_true = 0; % 平面总行程
for k=1:length(true_position)-1
    routeLong_2dim_true = routeLong_2dim_true + sqrt( (true_position(1,k+1)-true_position(1,k))^2 + (true_position(2,k+1)-true_position(2,k))^2 );
end
routeLong_2dim_nav = 0; % 平面总行程
for k=1:length(position_nav)-1
    routeLong_2dim_nav = routeLong_2dim_nav + sqrt( (position_nav(1,k+1)-position_nav(1,k))^2 + (position_nav(2,k+1)-position_nav(2,k))^2 );
end
routeError = routeLong_2dim_nav-routeLong_2dim_true ;
errorStr = sprintf('\t平面：\t真实行程：%0.5g m\t导航行程:%0.5g m\t行程误差：%0.5g m (%0.5g%%)\n',...
    routeLong_2dim_true,routeLong_2dim_nav,routeError,routeError/routeLong_2dim_true*100);
%% 计算最大误差
PosErrorLength = length(PosError);
maxAbsError_2dim = 0;   % 平面最大误差绝对值
for k=1:PosErrorLength
    error = sqrt( (PosError(1,k))^2+(PosError(2,k))^2 ) ;
    if error>maxAbsError_2dim
       maxAbsError_2dim = error ; 
    end
end
maxRelError_2dim = maxAbsError_2dim/routeLong_2dim_true ;    % 平面最大误差相对值

errorStr = sprintf('%s\t\t平面距原点最大误差：%0.5g m (%0.5g%%)\n',...
            errorStr,maxAbsError_2dim,maxRelError_2dim*100);
%% 计算终点误差
endAbsError_2dim = sqrt( (PosError(1,PosErrorLength))^2+(PosError(2,PosErrorLength))^2 ) ;  % 平面终点误差绝对值
endRelError_2dim = endAbsError_2dim/routeLong_2dim_true ;    % 平面终点误差相对值
errorStr = sprintf('%s\t\t平面终点位置误差：%0.5g m  (%0.5g%%) \n',...
            errorStr,endAbsError_2dim,endRelError_2dim*100 );
%% 计算空间三维位置误差及相对值
routeLong_3dim_true = 0; % 空间总行程
for k=1:length(true_position)-1
    routeLong_3dim_true = routeLong_3dim_true + sqrt( (true_position(1,k+1)-true_position(1,k))^2 + (true_position(2,k+1)-true_position(2,k))^2 + (true_position(3,k+1)-true_position(3,k))^2 );
end
routeLong_3dim_nav = 0; % 空间总行程
for k=1:length(position_nav)-1
    routeLong_3dim_nav = routeLong_3dim_nav + sqrt( (position_nav(1,k+1)-position_nav(1,k))^2 + (position_nav(2,k+1)-position_nav(2,k))^2 + (position_nav(3,k+1)-position_nav(3,k))^2 );
end
routeError = routeLong_3dim_nav-routeLong_3dim_true ;
errorStr = sprintf('%s\t空间：\t真实行程：%0.5g m\t导航行程:%0.5g m\t行程误差：%0.5g m (%0.5g%%)\n',...
        errorStr,routeLong_3dim_true,routeLong_3dim_nav,routeError,routeError/routeLong_3dim_true*100);
%% 计算终点误差
PosErrorLength = length(PosError);
endAbsError_3dim = sqrt( (PosError(1,PosErrorLength))^2+(PosError(2,PosErrorLength))^2+(PosError(3,PosErrorLength))^2 ) ;  % 空间终点误差绝对值
endRelError_3dim = endAbsError_3dim/routeLong_3dim_true ;    % 空间终点误差相对值
errorStr = sprintf('%s\t\t空间终点位置误差：%0.5g m  (%0.5g%%) \n',...
            errorStr,endAbsError_3dim,endRelError_3dim*100 );
%% 计算最大误差
maxAbsError_3dim = 0;   % 空间最大误差绝对值
for k=1:PosErrorLength
    error = sqrt( (PosError(1,k))^2+(PosError(2,k))^2+(PosError(3,k))^2 ) ;
    if error>maxAbsError_3dim
       maxAbsError_3dim = error ; 
    end
end
maxRelError_3dim = maxAbsError_3dim/routeLong_3dim_true ;    % 空间最大误差相对值
% 
errorStr = sprintf('%s\t\t空间 最大距远点最大误差：%0.5g m (%0.5g%%)\n',errorStr,...
    maxAbsError_3dim,maxRelError_3dim*100);

% 各维的误差
xyzLength_true = CalRouteLength( true_position ) ; % 得到各维的行程
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
errorStr = sprintf('%s\t各维 最大 位置误差(x、y、z)：(%0.5g,%0.5g,%0.5g)m\t(%0.5g%%,%0.5g%%,%0.5g%%)\n',...
                errorStr,maxPosError_x,maxPosError_y,maxPosError_z,maxPosError_x/xyzLength_true(1)*100,maxPosError_y/xyzLength_true(2)*100,maxPosError_z/xyzLength_true(3)*100);
endPosError_x = PosError(1,PosErrorLength) ;
endPosError_y = PosError(2,PosErrorLength) ;
endPosError_z = PosError(3,PosErrorLength) ;
errorStr = sprintf('%s\t各维 终点 位置误差(x、y、z)：(%0.5g,%0.5g,%0.5g)m\t(%0.5g%%,%0.5g%%,%0.5g%%)\n',...
                errorStr,endPosError_x,endPosError_y,endPosError_z,endPosError_x/xyzLength_true(1)*100,endPosError_y/xyzLength_true(2)*100,endPosError_z/xyzLength_true(3)*100);

%% 姿态各维最大误差
% if exist('AttitudeError','var')
%     maxAtdError_x = 0;
%     maxAtdError_y = 0;
%     maxAtdError_z = 0;
%     for k=1:PosErrorLength
%         if abs(AttitudeError(1,k))>abs(maxAtdError_x)
%            maxAtdError_x =  AttitudeError(1,k) ;
%         end
%         if abs(AttitudeError(2,k))>abs(maxAtdError_y)
%            maxAtdError_y =  AttitudeError(2,k) ;
%         end
%         if abs(AttitudeError(3,k))>abs(maxAtdError_z)
%            maxAtdError_z =  AttitudeError(3,k) ;
%         end
%     end
%     errorStr = sprintf('%s\t姿态最大误差 (俯仰、横滚、航向):(%0.5g,%0.5g,%0.5g)〞\n',...
%         errorStr,maxAtdError_x,maxAtdError_y,maxAtdError_z);
%     %% 姿态终点误差
%     finalAttitudeError = AttitudeError(:,length(AttitudeError));
%     errorStr = sprintf('%s\t姿态终点误差 (俯仰、横滚、航向):(%0.5g,%0.5g,%0.5g)〞\n',...
%             errorStr,finalAttitudeError(1),finalAttitudeError(2),finalAttitudeError(3));
% end


if exist('AttitudeError','var') && ~isempty(AttitudeError)
    AttitudeError = AttitudeError/3600 ;
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
    errorStr = sprintf('%s\t姿态最大误差 (俯仰、横滚、航向):(%0.5g,%0.5g,%0.5g)deg\n',...
        errorStr,maxAtdError_x,maxAtdError_y,maxAtdError_z);
    %% 姿态终点误差
    finalAttitudeError = AttitudeError(:,length(AttitudeError));
    errorStr = sprintf('%s\t姿态终点误差 (俯仰、横滚、航向):(%0.5g,%0.5g,%0.5g)deg\n',...
            errorStr,finalAttitudeError(1),finalAttitudeError(2),finalAttitudeError(3));
end

