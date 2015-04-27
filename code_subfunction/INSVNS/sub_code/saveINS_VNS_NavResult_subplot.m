% buaa xyz 2014.1.16

% 保存INS_VNS的组合导航结果为ResultDisplay模块特定格式（参考相关说明文档）

function INS_VNS_NavResult = saveINS_VNS_NavResult_subplot(integFre,combineFre,imu_fre,projectName,gp,isKnowTrue,trueTraeFre,...
    INTGpos,INTGvel,INTGatt,dPositionEsm,dVelocityEsm,dangleEsm,accDrift,gyroDrift,INTGPositionError,true_position,...
    INTGAttitudeError,true_attitude,INTGVelocityError,accDriftError,gyroDriftError,dangleEsmP,dVelocityEsmP,dPositionEsmP,...
    gyroDriftP,accDriftP,SINS_accError,X_pre_error,X_correct,gyroDriftEsmA,gyroDriftEsmAError,gyroDriftEsmB,gyroDriftEsmBError,diagP_gyroDRbbKFA,diagP_gyroDRbbUKFB)
% 以下部分也可直接拷贝到导航函数中，直接替代此函数的调用

% 存储为特定格式：每个变量一个细胞，包含成员：data，name,comment 和 dataFlag,frequency,project,subName
resultNum = 31;
INS_VNS_NavResult = cell(1,resultNum);

% 有4个共同的成员
for j=1:resultNum
    INS_VNS_NavResult{j}.dataFlag = 'xyz result display format';
    INS_VNS_NavResult{j}.frequency = integFre ;
    INS_VNS_NavResult{j}.project = projectName ;
    INS_VNS_NavResult{j}.subName = {'x(m)','y(m)','z(m)'};
end

INS_VNS_NavResult{1}.data = INTGpos ;
INS_VNS_NavResult{1}.name = 'position(m)';
INS_VNS_NavResult{1}.comment = '位置';

INS_VNS_NavResult{2}.data = INTGvel ;
INS_VNS_NavResult{2}.name = 'velocity(m/s)';
INS_VNS_NavResult{2}.comment = '速度';
INS_VNS_NavResult{2}.subName = {'x(m/s)','y(m/s)','z(m/s)'};

INS_VNS_NavResult{3}.data = INTGatt*180/pi ;   % 转为角度单位
INS_VNS_NavResult{3}.name = 'attitude(°)';
INS_VNS_NavResult{3}.comment = '姿态';
INS_VNS_NavResult{3}.subName = {'俯仰(°)','横滚(°)','航向(°)'};

INS_VNS_NavResult{4}.data = dPositionEsm ;
INS_VNS_NavResult{4}.name = 'positionErrorEstimate(m)';
INS_VNS_NavResult{4}.comment = '位置误差估计';

INS_VNS_NavResult{5}.data = dVelocityEsm ;
INS_VNS_NavResult{5}.name = 'velocityErrorEstimate(m/s)';
INS_VNS_NavResult{5}.comment = '速度误差估计';
INS_VNS_NavResult{5}.subName = {'x(m/s)','y(m/s)','z(m/s)'};

INS_VNS_NavResult{6}.data = dangleEsm*180/pi*3600 ;     % 转为角秒单位
INS_VNS_NavResult{6}.name = 'attitudeErrorEstimate(‘’)';
INS_VNS_NavResult{6}.comment = '平台误差角估计';
INS_VNS_NavResult{6}.subName = {'俯仰(‘’)','横滚(‘’)','航向(‘’)'};

INS_VNS_NavResult{7}.data = accDrift/(gp*1e-6) ;     %
INS_VNS_NavResult{7}.name = 'accDrift(ug)';
INS_VNS_NavResult{7}.comment = '加计常值漂移估计';
INS_VNS_NavResult{7}.subName = {'x(ug)','y(ug)','z(ug)'};
meanAccDrift = mean(accDrift/(gp*1e-6),2);  % 加计常值漂移估计均值
meanAccDriftText{1} = '均值';
meanAccDriftText{2} = sprintf('x：%0.3ug',meanAccDrift(1));
meanAccDriftText{3} = sprintf('y：%0.3ug',meanAccDrift(2));
meanAccDriftText{4} = sprintf('z：%0.3ug',meanAccDrift(3));
INS_VNS_NavResult{7}.text = meanAccDriftText ;

INS_VNS_NavResult{8}.data = gyroDrift*180/pi*3600 ;     % rad/s 转化为 °/h
INS_VNS_NavResult{8}.name = 'gyroDrift(°/h)';
INS_VNS_NavResult{8}.comment = '陀螺常值漂移估计';
INS_VNS_NavResult{8}.subName = {'x(°/h)','y(°/h)','z(°/h)'};
meanGyroDrift = mean(gyroDrift*180/pi*3600,2);  % 陀螺常值漂移估计均值
meanGyroDriftText{1} = '均值';
meanGyroDriftText{2} = sprintf('x：%0.3f°/h',meanGyroDrift(1));
meanGyroDriftText{3} = sprintf('y：%0.3f°/h',meanGyroDrift(2));
meanGyroDriftText{4} = sprintf('z：%0.3f°/h',meanGyroDrift(3));
INS_VNS_NavResult{8}.text = meanGyroDriftText ;

INS_VNS_NavResult{9}.data = X_correct(1:3,:) ;    
INS_VNS_NavResult{9}.name = 'dangleFilterCorrect(‘’)';
INS_VNS_NavResult{9}.comment = '平台失准角滤波修正量';
INS_VNS_NavResult{9}.frequency = integFre ;
INS_VNS_NavResult{9}.subName = {'俯仰(‘’)','横滚(‘’)','航向(‘’)'};

INS_VNS_NavResult{10}.data = X_correct(4:6,:) ;    
INS_VNS_NavResult{10}.name = 'dVelocityFilterCorrect(‘’)';
INS_VNS_NavResult{10}.comment = '速度滤波修正量';
INS_VNS_NavResult{10}.frequency = integFre ;
INS_VNS_NavResult{10}.subName = {'x(m/s)','y(m/s)','z(m/s)'};

INS_VNS_NavResult{11}.data = X_correct(7:9,:) ;    
INS_VNS_NavResult{11}.name = 'dPositionFilterCorrect(‘’)';
INS_VNS_NavResult{11}.comment = '位置滤波修正量';
INS_VNS_NavResult{11}.frequency = integFre ;
    
INS_VNS_NavResult{12}.data = gyroDriftEsmA*180/pi*3600 ;     % rad/s 转化为 °/h
INS_VNS_NavResult{12}.name = 'gyroDriftEsmA(°/h)';
INS_VNS_NavResult{12}.comment = 'DRbb_A陀螺常值漂移估计';
INS_VNS_NavResult{12}.subName = {'x(°/h)','y(°/h)','z(°/h)'};
meanGyroDrift = mean(gyroDriftEsmA*180/pi*3600,2);  % 陀螺常值漂移估计均值
meanGyroDriftText{1} = '均值';
meanGyroDriftText{2} = sprintf('x：%0.3f°/h',meanGyroDrift(1));
meanGyroDriftText{3} = sprintf('y：%0.3f°/h',meanGyroDrift(2));
meanGyroDriftText{4} = sprintf('z：%0.3f°/h',meanGyroDrift(3));
INS_VNS_NavResult{12}.text = meanGyroDriftText ;

INS_VNS_NavResult{13}.data = gyroDriftEsmB*180/pi*3600 ;     % rad/s 转化为 °/h
INS_VNS_NavResult{13}.name = 'gyroDriftEsmB(°/h)';
INS_VNS_NavResult{13}.comment = 'DRbb_B陀螺常值漂移估计';
INS_VNS_NavResult{13}.subName = {'x(°/h)','y(°/h)','z(°/h)'};
meanGyroDrift = mean(gyroDriftEsmB*180/pi*3600,2);  % 陀螺常值漂移估计均值
meanGyroDriftText{1} = '均值';
meanGyroDriftText{2} = sprintf('x：%0.3f°/h',meanGyroDrift(1));
meanGyroDriftText{3} = sprintf('y：%0.3f°/h',meanGyroDrift(2));
meanGyroDriftText{4} = sprintf('z：%0.3f°/h',meanGyroDrift(3));
INS_VNS_NavResult{13}.text = meanGyroDriftText ;

INS_VNS_NavResult{14}.data = diagP_gyroDRbbKFA ;    
INS_VNS_NavResult{14}.name = 'diagP_gyroDRbbKFA';
INS_VNS_NavResult{14}.comment = 'DRbb_A陀螺常值漂移估计方差';
INS_VNS_NavResult{14}.frequency = integFre ;

INS_VNS_NavResult{15}.data = diagP_gyroDRbbUKFB ;    
INS_VNS_NavResult{15}.name = 'diagP_gyroDRbbUKFB';
INS_VNS_NavResult{15}.comment = 'DRbb_B陀螺常值漂移估计方差';
INS_VNS_NavResult{15}.frequency = integFre ;

frontNum = 15;
for j=frontNum+1:resultNum
    INS_VNS_NavResult{j}.frequency = combineFre ;
end
if isKnowTrue==1
    INS_VNS_NavResult{frontNum+1}.data = INTGPositionError;
    INS_VNS_NavResult{frontNum+1}.name = 'positionError(m)';
    INS_VNS_NavResult{frontNum+1}.comment = '位置误差';    
    % 计算最大相对误差，终点相对误差
    validLength = fix(length(INTGpos)*(trueTraeFre/combineFre));
    true_position_valid = true_position(:,1:validLength) ;
    text_error_xyz = GetErrorText( true_position_valid,INTGPositionError ) ;
    INS_VNS_NavResult{frontNum+1}.text = text_error_xyz ;
    
    INS_VNS_NavResult{frontNum+2}.data = INTGAttitudeError*180/pi*3600;
    INS_VNS_NavResult{frontNum+2}.name = 'attitudeError(‘’)';
    INS_VNS_NavResult{frontNum+2}.comment = '姿态误差';
    INS_VNS_NavResult{frontNum+2}.subName = {'俯仰(‘’)','横滚(‘’)','航向(‘’)'};
    % 计算最大相对误差，终点相对误差
    validLength = fix(length(INTGatt)*(trueTraeFre/combineFre));
    true_attitude_valid = true_attitude(:,1:validLength) ;
    text_error_xyz = GetErrorText( true_attitude_valid,INTGAttitudeError*180/pi*3600 ) ;
    INS_VNS_NavResult{frontNum+2}.text = text_error_xyz ;
    
    INS_VNS_NavResult{frontNum+3}.data = INTGVelocityError;
    INS_VNS_NavResult{frontNum+3}.name = 'velocityError(m/ｓ)';
    INS_VNS_NavResult{frontNum+3}.comment = '速度误差';
    INS_VNS_NavResult{frontNum+3}.subName = {'x(m/s)','y(m/s)','z(m/s)'};
    
    INS_VNS_NavResult{frontNum+4}.data = accDriftError/(gp*1e-6) ;     % 转换为 ug 误差
    INS_VNS_NavResult{frontNum+4}.name = 'accDriftError(ug)';
    INS_VNS_NavResult{frontNum+4}.comment = '加计常值漂移估计误差';
    INS_VNS_NavResult{frontNum+4}.subName = {'x(ug)','y(ug)','z(ug)'};
    
    INS_VNS_NavResult{frontNum+5}.data = gyroDriftError*180/pi*3600 ;     % 转换为 °/h 
    INS_VNS_NavResult{frontNum+5}.name = 'gyroDriftError(°/h)';
    INS_VNS_NavResult{frontNum+5}.comment = '陀螺常值漂移估计误差';
    INS_VNS_NavResult{frontNum+5}.subName = {'x(°/h)','y(°/h)','z(°/h)'};
    
    INS_VNS_NavResult{frontNum+6}.data = dangleEsmP*180/pi*3600 ;     % 转换为 ‘’
    INS_VNS_NavResult{frontNum+6}.name = 'dangleEsmP(‘’)';
    INS_VNS_NavResult{frontNum+6}.comment = '平台误差角估计均方差';
    INS_VNS_NavResult{frontNum+6}.frequency = integFre ;
    INS_VNS_NavResult{frontNum+6}.subName = {'x(‘’)','y(‘’)','z(‘’)'};
    
    INS_VNS_NavResult{frontNum+7}.data = dVelocityEsmP ;     
    INS_VNS_NavResult{frontNum+7}.name = 'dVelocityEsmP(m/ｓ)';
    INS_VNS_NavResult{frontNum+7}.comment = '速度误差估计均方差';
    INS_VNS_NavResult{frontNum+7}.frequency = integFre ;
    INS_VNS_NavResult{frontNum+7}.subName = {'x(m/s)','y(m/s)','z(m/s)'};
    
    INS_VNS_NavResult{frontNum+8}.data = dPositionEsmP ;     
    INS_VNS_NavResult{frontNum+8}.name = 'dPositionEsmP(m)';
    INS_VNS_NavResult{frontNum+8}.comment = '位置误差估计均方差';
    INS_VNS_NavResult{frontNum+8}.frequency = integFre ;
    
    INS_VNS_NavResult{frontNum+9}.data = gyroDriftP*180/pi*3600 ;     % 转换为 °/h 
    INS_VNS_NavResult{frontNum+9}.name = 'gyroDriftP(m)';
    INS_VNS_NavResult{frontNum+9}.comment = '陀螺漂移估计均方差';
    INS_VNS_NavResult{frontNum+9}.frequency = integFre ;
    INS_VNS_NavResult{frontNum+9}.subName = {'x(°/h)','y(°/h)','z(°/h)'};
    
    INS_VNS_NavResult{frontNum+10}.data = accDriftP/(gp*1e-6) ;     % 转换为 °/h 
    INS_VNS_NavResult{frontNum+10}.name = 'accDriftP(ug)';
    INS_VNS_NavResult{frontNum+10}.comment = '加计漂移估计均方差';
    INS_VNS_NavResult{frontNum+10}.frequency = integFre ;
    INS_VNS_NavResult{frontNum+10}.subName = {'x(ug)','y(ug)','z(ug)'};
    
    INS_VNS_NavResult{frontNum+11}.data = SINS_accError/(gp*1e-6) ;     % 转换为 ug
    INS_VNS_NavResult{frontNum+11}.name = 'SINS_accError(ug)';
    INS_VNS_NavResult{frontNum+11}.comment = 'SINS解算加速度误差';
    INS_VNS_NavResult{frontNum+11}.frequency = imu_fre ;
    INS_VNS_NavResult{frontNum+11}.subName = {'x(ug)','y(ug)','z(ug)'};
    
    %% 状态估计误差
    
    INS_VNS_NavResult{frontNum+12}.data = X_pre_error(1:3,:) ;    
    INS_VNS_NavResult{frontNum+12}.name = 'dangleEsmError(‘’)';
    INS_VNS_NavResult{frontNum+12}.comment = '平台失准角估计误差';
    INS_VNS_NavResult{frontNum+12}.frequency = integFre ;
    INS_VNS_NavResult{frontNum+12}.subName = {'俯仰(‘’)','横滚(‘’)','航向(‘’)'};
    
    INS_VNS_NavResult{frontNum+13}.data = X_pre_error(4:6,:) ;    
    INS_VNS_NavResult{frontNum+13}.name = 'dVelocityEsmError(‘’)';
    INS_VNS_NavResult{frontNum+13}.comment = '速度估计误差';
    INS_VNS_NavResult{frontNum+13}.frequency = integFre ;
    INS_VNS_NavResult{frontNum+13}.subName = {'x(m/s)','y(m/s)','z(m/s)'};
    
    INS_VNS_NavResult{frontNum+14}.data = X_pre_error(7:9,:) ;    
    INS_VNS_NavResult{frontNum+14}.name = 'dPositionEsmError(‘’)';
    INS_VNS_NavResult{frontNum+14}.comment = '位置估计误差';
    INS_VNS_NavResult{frontNum+14}.frequency = integFre ;
    
    INS_VNS_NavResult{frontNum+15}.data = gyroDriftEsmAError*180/pi*3600 ;     % 转换为 °/h 
    INS_VNS_NavResult{frontNum+15}.name = 'gyroDriftEsmAError(°/h)';
    INS_VNS_NavResult{frontNum+15}.comment = 'DRbb_A陀螺常值漂移估计误差';
    INS_VNS_NavResult{frontNum+15}.subName = {'x(°/h)','y(°/h)','z(°/h)'};
    
    INS_VNS_NavResult{frontNum+16}.data = gyroDriftEsmBError*180/pi*3600 ;     % 转换为 °/h 
    INS_VNS_NavResult{frontNum+16}.name = 'gyroDriftEsmBError(°/h)';
    INS_VNS_NavResult{frontNum+16}.comment = 'DRbb_B陀螺常值漂移估计误差';
    INS_VNS_NavResult{frontNum+16}.subName = {'x(°/h)','y(°/h)','z(°/h)'};
end
if length(INS_VNS_NavResult)~=resultNum
    errordlg(['resultNum设置出错！请重新填写为:',num2str(length(INS_VNS_NavResult))])
end