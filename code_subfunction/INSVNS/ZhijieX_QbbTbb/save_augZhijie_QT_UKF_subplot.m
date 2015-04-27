% buaa xyz 2014.5.4

% 保存INS_VNS的组合导航结果为ResultDisplay模块特定格式（参考相关说明文档）

function INS_VNS_NavResult = save_augZhijie_QT_UKF_subplot(integFre,combineFre,imu_fre,projectName,gp,isKnowTrue,trueTraeFre,...
    INTGpos,INTGvel,INTGacc,INTGatt,accDrift,gyroDrift,INTGPositionError,true_position,...
    INTGAttitudeError,true_attitude,INTGVelocityError,INTGaccError,accDriftError,gyroDriftError,angleEsmP,velocityEsmP,positionEsmP,...
    gyroDriftP,accDriftP,SINS_accError,X_correct,Zinteg_error,Zinteg_pre,Zinteg )
% 以下部分也可直接拷贝到导航函数中，直接替代此函数的调用

% 存储为特定格式：每个变量一个细胞，包含成员：data，name,comment 和 dataFlag,frequency,project,subName
resultNum = 40;
INS_VNS_NavResult = cell(1,resultNum);

% 有4个共同的成员
for j=1:resultNum
    INS_VNS_NavResult{j}.dataFlag = 'xyz result display format';
    INS_VNS_NavResult{j}.frequency = integFre ;
    INS_VNS_NavResult{j}.project = projectName ;
    INS_VNS_NavResult{j}.subName = {'x(m)','y(m)','z(m)'};
end

resultN = 1;
INS_VNS_NavResult{resultN}.data = INTGpos ;
INS_VNS_NavResult{resultN}.name = 'position(m)';
INS_VNS_NavResult{resultN}.comment = '位置';

resultN = resultN+1;
INS_VNS_NavResult{resultN}.data = INTGvel ;
INS_VNS_NavResult{resultN}.name = 'velocity(m/s)';
INS_VNS_NavResult{resultN}.comment = '速度';
INS_VNS_NavResult{resultN}.subName = {'x(m/s)','y(m/s)','z(m/s)'};

resultN = resultN+1;
INS_VNS_NavResult{resultN}.data = INTGacc ;
INS_VNS_NavResult{resultN}.name = 'acc(m/s)';
INS_VNS_NavResult{resultN}.comment = '加速度';
INS_VNS_NavResult{resultN}.subName = {'x(m/s^2)','y(m/s^2)','z(m/s^2)'};

resultN = resultN+1;
INS_VNS_NavResult{resultN}.data = INTGatt*180/pi ;   % 转为角度单位
INS_VNS_NavResult{resultN}.name = 'attitude(°)';
INS_VNS_NavResult{resultN}.comment = '姿态';
INS_VNS_NavResult{resultN}.subName = {'俯仰(°)','横滚(°)','航向(°)'};

% resultN = resultN+1;
% INS_VNS_NavResult{resultN}.data = dPositionEsm ;
% INS_VNS_NavResult{resultN}.name = 'positionErrorEstimate(m)';
% INS_VNS_NavResult{resultN}.comment = '位置误差估计';
% 
% resultN = resultN+1;
% INS_VNS_NavResult{resultN}.data = dVelocityEsm ;
% INS_VNS_NavResult{resultN}.name = 'velocityErrorEstimate(m/s)';
% INS_VNS_NavResult{resultN}.comment = '速度误差估计';
% INS_VNS_NavResult{resultN}.subName = {'x(m/s)','y(m/s)','z(m/s)'};
% 
% if ~isempty(dangleEsm)
%     resultN = resultN+1;
%     INS_VNS_NavResult{resultN}.data = dangleEsm*180/pi*3600 ;     % 转为角秒单位
%     INS_VNS_NavResult{resultN}.name = 'attitudeErrorEstimate(‘’)';
%     INS_VNS_NavResult{resultN}.comment = '平台误差角估计';
%     INS_VNS_NavResult{resultN}.subName = {'俯仰(‘’)','横滚(‘’)','航向(‘’)'};
% end

resultN = resultN+1;
INS_VNS_NavResult{resultN}.data = accDrift/(gp*1e-6) ;     %
INS_VNS_NavResult{resultN}.name = 'accDrift(ug)';
INS_VNS_NavResult{resultN}.comment = '加计常值漂移估计';
INS_VNS_NavResult{resultN}.subName = {'x(ug)','y(ug)','z(ug)'};
meanAccDrift = mean(accDrift/(gp*1e-6),2);  % 加计常值漂移估计均值
meanAccDriftText{1} = '均值';
meanAccDriftText{2} = sprintf('x：%0.3ug',meanAccDrift(1));
meanAccDriftText{3} = sprintf('y：%0.3ug',meanAccDrift(2));
meanAccDriftText{4} = sprintf('z：%0.3ug',meanAccDrift(3));
INS_VNS_NavResult{resultN}.text = meanAccDriftText ;

resultN = resultN+1;
INS_VNS_NavResult{resultN}.data = gyroDrift*180/pi*3600 ;     % rad/s 转化为 °/h
INS_VNS_NavResult{resultN}.name = 'gyroDrift(°/h)';
INS_VNS_NavResult{resultN}.comment = '陀螺常值漂移估计';
INS_VNS_NavResult{resultN}.subName = {'x(°/h)','y(°/h)','z(°/h)'};
meanGyroDrift = mean(gyroDrift*180/pi*3600,2);  % 陀螺常值漂移估计均值
meanGyroDriftText{1} = '均值';
meanGyroDriftText{2} = sprintf('x：%0.3f°/h',meanGyroDrift(1));
meanGyroDriftText{3} = sprintf('y：%0.3f°/h',meanGyroDrift(2));
meanGyroDriftText{4} = sprintf('z：%0.3f°/h',meanGyroDrift(3));
INS_VNS_NavResult{resultN}.text = meanGyroDriftText ;

if ~isempty(X_correct)
%     resultN = resultN+1;
%     INS_VNS_NavResult{resultN}.data = X_correct(1:4,:) ;    
%     INS_VNS_NavResult{resultN}.name = 'dangleFilterCorrect(‘’)';
%     INS_VNS_NavResult{resultN}.comment = '平台失准角滤波修正量';
%     INS_VNS_NavResult{resultN}.frequency = integFre ;
%     INS_VNS_NavResult{resultN}.subName = {'俯仰(‘’)','横滚(‘’)','航向(‘’)'};

    resultN = resultN+1;
    INS_VNS_NavResult{resultN}.data = X_correct(5:7,:) ;    
    INS_VNS_NavResult{resultN}.name = 'dVelocityFilterCorrect(‘’)';
    INS_VNS_NavResult{resultN}.comment = '速度滤波修正量';
    INS_VNS_NavResult{resultN}.frequency = integFre ;
    INS_VNS_NavResult{resultN}.subName = {'x(m/s)','y(m/s)','z(m/s)'};

    resultN = resultN+1;
    INS_VNS_NavResult{resultN}.data = X_correct(8:10,:) ;    
    INS_VNS_NavResult{resultN}.name = 'dPositionFilterCorrect(‘’)';
    INS_VNS_NavResult{resultN}.comment = '位置滤波修正量';
    INS_VNS_NavResult{resultN}.frequency = integFre ;
end
if ~isempty(Zinteg_error)
    resultN = resultN+1;
    INS_VNS_NavResult{resultN}.data = Zinteg_error(1:4,:) ;    
    INS_VNS_NavResult{resultN}.name = 'Qbb_error';
    INS_VNS_NavResult{resultN}.comment = 'Qbb偏差';
    INS_VNS_NavResult{resultN}.frequency = integFre ;
    INS_VNS_NavResult{resultN}.subName = {'q0','q1','q2','q3'};
    
    resultN = resultN+1;
    INS_VNS_NavResult{resultN}.data = Zinteg_error(5:7,:) ;    
    INS_VNS_NavResult{resultN}.name = 'Tbb_error';
    INS_VNS_NavResult{resultN}.comment = 'Tbb偏差';
    INS_VNS_NavResult{resultN}.frequency = integFre ;
end

if ~isempty(Zinteg_pre)
    resultN = resultN+1;
    INS_VNS_NavResult{resultN}.data = Zinteg_pre(1:4,:) ;    
    INS_VNS_NavResult{resultN}.name = 'Qbb_pre';
    INS_VNS_NavResult{resultN}.comment = 'Qbb一步预测';
    INS_VNS_NavResult{resultN}.frequency = integFre ;
    INS_VNS_NavResult{resultN}.subName = {'q0','q1','q2','q3'};
    
    resultN = resultN+1;
    INS_VNS_NavResult{resultN}.data = Zinteg_pre(5:7,:) ;    
    INS_VNS_NavResult{resultN}.name = 'Tbb_pre';
    INS_VNS_NavResult{resultN}.comment = 'Tbb一步预测';
    INS_VNS_NavResult{resultN}.frequency = integFre ;
end

if ~isempty(Zinteg)
    resultN = resultN+1;
    INS_VNS_NavResult{resultN}.data = Zinteg(1:4,:) ;    
    INS_VNS_NavResult{resultN}.name = 'Qbb';
    INS_VNS_NavResult{resultN}.comment = '量测量Qbb';
    INS_VNS_NavResult{resultN}.frequency = integFre ;
    INS_VNS_NavResult{resultN}.subName = {'q0','q1','q2','q3'};
    
    resultN = resultN+1;
    INS_VNS_NavResult{resultN}.data = Zinteg(5:7,:) ;    
    INS_VNS_NavResult{resultN}.name = 'Tbb';
    INS_VNS_NavResult{resultN}.comment = '量测量Tbb';
    INS_VNS_NavResult{resultN}.frequency = integFre ;
end

for j=resultN:resultNum
    INS_VNS_NavResult{j}.frequency = combineFre ;
end
if isKnowTrue==1
    resultN = resultN+1;
    INS_VNS_NavResult{resultN}.data = INTGPositionError;
    INS_VNS_NavResult{resultN}.name = 'positionError(m)';
    INS_VNS_NavResult{resultN}.comment = '位置误差';    
    % 计算最大相对误差，终点相对误差
    validLength = fix((length(INTGpos)-1)*(trueTraeFre/combineFre))+1 ;
    true_position_valid = true_position(:,1:validLength) ;
    text_error_xyz = GetErrorText( true_position_valid,INTGPositionError ) ;
    INS_VNS_NavResult{resultN}.text = text_error_xyz ;
    
    resultN = resultN+1;
    INS_VNS_NavResult{resultN}.data = INTGAttitudeError*180/pi;
    INS_VNS_NavResult{resultN}.name = 'attitudeError(°)';
    INS_VNS_NavResult{resultN}.comment = '姿态误差';
    INS_VNS_NavResult{resultN}.subName = {'俯仰(°)','横滚(°)','航向(°)'};
    % 计算最大相对误差，终点相对误差
    validLength = fix((length(INTGatt)-1)*(trueTraeFre/combineFre))+1;
    true_attitude_valid = true_attitude(:,1:validLength) ;
    text_error_xyz = GetErrorText( true_attitude_valid,INTGAttitudeError*180/pi ) ;
    INS_VNS_NavResult{resultN}.text = text_error_xyz ;
    
    resultN = resultN+1;
    INS_VNS_NavResult{resultN}.data = INTGVelocityError;
    INS_VNS_NavResult{resultN}.name = 'velocityError(m/ｓ)';
    INS_VNS_NavResult{resultN}.comment = '速度误差';
    INS_VNS_NavResult{resultN}.subName = {'x(m/s)','y(m/s)','z(m/s)'};
    
    resultN = resultN+1;
    INS_VNS_NavResult{resultN}.data = INTGaccError;
    INS_VNS_NavResult{resultN}.name = 'accError(m/ｓ)';
    INS_VNS_NavResult{resultN}.comment = '加速度误差';
    INS_VNS_NavResult{resultN}.subName = {'x(m/s^2)','y(m/s^2)','z(m/s^2)'};
        
    resultN = resultN+1;
    INS_VNS_NavResult{resultN}.data = accDriftError/(gp*1e-6) ;     % 转换为 ug 误差
    INS_VNS_NavResult{resultN}.name = 'accDriftError(ug)';
    INS_VNS_NavResult{resultN}.comment = '加计常值漂移估计误差';
    INS_VNS_NavResult{resultN}.subName = {'x(ug)','y(ug)','z(ug)'};
    
    resultN = resultN+1;
    INS_VNS_NavResult{resultN}.data = gyroDriftError*180/pi*3600 ;     % 转换为 °/h 
    INS_VNS_NavResult{resultN}.name = 'gyroDriftError(°/h)';
    INS_VNS_NavResult{resultN}.comment = '陀螺常值漂移估计误差';
    INS_VNS_NavResult{resultN}.subName = {'x(°/h)','y(°/h)','z(°/h)'};
    
    resultN = resultN+1;
    INS_VNS_NavResult{resultN}.data = angleEsmP*180/pi ;     % 转换为 ‘’
    INS_VNS_NavResult{resultN}.name = 'angleEsmP(°)';
    INS_VNS_NavResult{resultN}.comment = '姿态角估计均方差';
    INS_VNS_NavResult{resultN}.frequency = integFre ;
    INS_VNS_NavResult{resultN}.subName = {'x(°)','y(°)','z(°)'};
    
    resultN = resultN+1;
    INS_VNS_NavResult{resultN}.data = velocityEsmP ;     
    INS_VNS_NavResult{resultN}.name = 'velocityEsmP(m/ｓ)';
    INS_VNS_NavResult{resultN}.comment = '速度估计均方差';
    INS_VNS_NavResult{resultN}.frequency = integFre ;
    INS_VNS_NavResult{resultN}.subName = {'x(m/s)','y(m/s)','z(m/s)'};
    
    resultN = resultN+1;
    INS_VNS_NavResult{resultN}.data = positionEsmP ;     
    INS_VNS_NavResult{resultN}.name = 'positionEsmP(m)';
    INS_VNS_NavResult{resultN}.comment = '位置估计均方差';
    INS_VNS_NavResult{resultN}.frequency = integFre ;
    
    resultN = resultN+1;
    INS_VNS_NavResult{resultN}.data = gyroDriftP*180/pi*3600 ;     % 转换为 °/h 
    INS_VNS_NavResult{resultN}.name = 'gyroDriftP(m)';
    INS_VNS_NavResult{resultN}.comment = '陀螺漂移估计均方差';
    INS_VNS_NavResult{resultN}.frequency = integFre ;
    INS_VNS_NavResult{resultN}.subName = {'x(°/h)','y(°/h)','z(°/h)'};
    
    resultN = resultN+1;
    INS_VNS_NavResult{resultN}.data = accDriftP/(gp*1e-6) ;     % 转换为 °/h 
    INS_VNS_NavResult{resultN}.name = 'accDriftP(ug)';
    INS_VNS_NavResult{resultN}.comment = '加计漂移估计均方差';
    INS_VNS_NavResult{resultN}.frequency = integFre ;
    INS_VNS_NavResult{resultN}.subName = {'x(ug)','y(ug)','z(ug)'};
    
    resultN = resultN+1;
    INS_VNS_NavResult{resultN}.data = SINS_accError/(gp*1e-6) ;     % 转换为 ug
    INS_VNS_NavResult{resultN}.name = 'SINS_accError(ug)';
    INS_VNS_NavResult{resultN}.comment = 'SINS解算加速度误差';
    INS_VNS_NavResult{resultN}.frequency = imu_fre ;
    INS_VNS_NavResult{resultN}.subName = {'x(ug)','y(ug)','z(ug)'};
    
    %% 状态估计误差
%     if ~isempty(X_pre)
%         resultN = resultN+1;
%         q_pre = X_pre(1:4,:) ;
%         attitude_pre = zeors(3,length(q_pre));
%         opintions.headingScope=180;
%         for n=1:length(q_pre)
%             Crb = FQtoCnb(q_pre);
%             attitude_pre(:,n) = GetAttitude(Crb,'rad',opintions);
%         end        
%         INS_VNS_NavResult{resultN}.data = attitude_pre ;    
%         INS_VNS_NavResult{resultN}.name = 'attitudepre(rad)';
%         INS_VNS_NavResult{resultN}.comment = '姿态角一步预测';
%         INS_VNS_NavResult{resultN}.frequency = integFre ;
%         INS_VNS_NavResult{resultN}.subName = {'俯仰(‘’)','横滚(‘’)','航向(‘’)'};
%         
%         resultN = resultN+1;
%         INS_VNS_NavResult{resultN}.data = X_pre(5:7,:) ;    
%         INS_VNS_NavResult{resultN}.name = 'positionPre(m)';
%         INS_VNS_NavResult{resultN}.comment = '位置一步预测';
%         INS_VNS_NavResult{resultN}.frequency = integFre ;
%         
%         resultN = resultN+1;
%         INS_VNS_NavResult{resultN}.data = X_pre(8:10,:) ;    
%         INS_VNS_NavResult{resultN}.name = 'velocityPre(m/s)';
%         INS_VNS_NavResult{resultN}.comment = '速度一步预测';
%         INS_VNS_NavResult{resultN}.frequency = integFre ;
%         INS_VNS_NavResult{resultN}.subName = {'x(m/s)','y(m/s)','z(m/s)'};
% 
%     end
end

INS_VNS_NavResult = INS_VNS_NavResult(1:resultN);
