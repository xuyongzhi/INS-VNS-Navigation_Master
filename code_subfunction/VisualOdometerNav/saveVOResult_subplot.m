% buaa xyz 2014.1.16

% 保存INS_VNS的组合导航结果为ResultDisplay模块特定格式（参考相关说明文档）

function VOResult = saveVOResult_subplot( isKnowTrue,frequency_VO,VOsta,VOsta_trueRbb,VOpos,VOvel,matchedNum,aveFeatureNum,...
    VOposError,VOvelError, VOstaError,VOsta_trueRbb_error,VOsta_trueTbb_error,combineFre,trueTraeFre,true_position,VOStaStepError,...
    VOStaStepError_A,VOStaStepError_B,VOCrbError,VOCrcError,VOrcAngle,VOStaStepError_Adefine,angle_bb,Tbb,true_angle_bb,trueTbb,AngleError,TbbError,runTime_image )
resultNum = 30;
VOResult = cell(1,resultNum);

% 有4个共同的成员
for j=1:resultNum
    VOResult{j}.dataFlag = 'xyz result display format';
    VOResult{j}.frequency = frequency_VO ;
    VOResult{j}.project = 'VO';
    VOResult{j}.subName = {'x(m)','y(m)','z(m)'};
%     if ~isempty(runTime_image)
%         VOResult{j}.runTime = runTime_image ;
%     end
end
result_k = 0;
result_k = result_k+1;
VOResult{result_k}.data = VOsta;
VOResult{result_k}.name = 'position(m)';
VOResult{result_k}.comment = '位置';

result_k = result_k+1;
VOResult{result_k}.data = VOsta_trueRbb;
VOResult{result_k}.name = 'position_trueAttitude(m)';
VOResult{result_k}.comment = '无姿态误差_位置';


result_k = result_k+1;
VOResult{result_k}.data = VOpos*180/pi ;   % 转为角度单位
VOResult{result_k}.name = 'attitude(°)';
VOResult{result_k}.comment = '姿态';
VOResult{result_k}.subName = {'俯仰(°)','横滚(°)','航向(°)'};

result_k = result_k+1;
VOResult{result_k}.data = VOvel;
VOResult{result_k}.name = 'velocity(m/s)';
VOResult{result_k}.comment = '速度';
VOResult{result_k}.subName = {'x(m/s)','y(m/s)','z(m/s)'};

result_k = result_k+1;
VOResult{result_k}.data = Tbb;
VOResult{result_k}.name = 'Tbb(m)';
VOResult{result_k}.comment = '视觉Tbb';

result_k = result_k+1;
VOResult{result_k}.data = trueTbb;
VOResult{result_k}.name = 'Tbb(m)';
VOResult{result_k}.comment = '真实Tbb';
VOResult{result_k}.project = 'true';

result_k = result_k+1;
VOResult{result_k}.data = TbbError;
VOResult{result_k}.name = 'TbbError(m)';
VOResult{result_k}.comment = 'Tbb误差';

result_k = result_k+1;
VOResult{result_k}.data = angle_bb*180/pi ;   % 转为角度单位
VOResult{result_k}.name = 'angle_bb(°)';
VOResult{result_k}.comment = '视觉Rbb欧拉角';
VOResult{result_k}.subName = {'俯仰(°)','横滚(°)','航向(°)'};

result_k = result_k+1;
VOResult{result_k}.data = true_angle_bb*180/pi ;   % 转为角度单位
VOResult{result_k}.name = 'angle_bb(°)';
VOResult{result_k}.comment = '真实Rbb欧拉角';
VOResult{result_k}.subName = {'俯仰(°)','横滚(°)','航向(°)'};
VOResult{result_k}.project = 'true';

result_k = result_k+1;
VOResult{result_k}.data = AngleError*180/pi ;   % 转为角度单位
VOResult{result_k}.name = 'angle_bb_error(°)';
VOResult{result_k}.comment = 'Rbb欧拉角误差';
VOResult{result_k}.subName = {'俯仰(°)','横滚(°)','航向(°)'};


if ~isempty(matchedNum)
    if ~isempty(aveFeatureNum)
        result_k = result_k+1;
        rat = zeros(size(matchedNum));
        for k=1:length(matchedNum)
            rat(k)=matchedNum(k)/aveFeatureNum(k)*100;
        end
        VOResult{result_k}.data = [matchedNum;aveFeatureNum;rat];
        VOResult{result_k}.name = 'matchedNum';
        VOResult{result_k}.comment = '特征点个数' ;
        VOResult{result_k}.subName = {'匹配成功个数','四图平均个数','平均匹配百分比(%)'};
    else
        result_k = result_k+1;
        VOResult{result_k}.data = matchedNum;
        VOResult{result_k}.name = 'matchedNum';
        VOResult{result_k}.comment = '特征点个数' ;
    end
end


% VOsta_trueRbb=[];
% VOvel=[];
% VOvelError=[];

% VOStaStepError=[];
% VOStaStepError_A=[];
% VOStaStepError_B=[];
% VOCrbError=[];
% VOCrcError=[];
% VOrcAngle=[];
% VOStaStepError_Adefine=[];
% angle_bb=[];
% true_angle_bb=[];
% trueTbb=[];

Tbb_sel=[];

if  isKnowTrue==1
    result_k = result_k+1;
    VOResult{result_k}.data = VOposError*180/pi;
    VOResult{result_k}.name = 'attitudeError(°)';
    VOResult{result_k}.comment = '姿态误差';
    VOResult{result_k}.subName = {'俯仰(°)','横滚(°)','航向(°)'};
    VOResult{result_k}.frequency = combineFre ;
    
    result_k = result_k+1;
    VOResult{result_k}.data = VOvelError;
    VOResult{result_k}.name = 'velocityError(m/ｓ)';
    VOResult{result_k}.comment = '速度误差';
    VOResult{result_k}.frequency = combineFre ;
    VOResult{result_k}.subName = {'x(m/s)','y(m/s)','z(m/s)'};
    
    result_k = result_k+1;
    VOResult{result_k}.data = VOstaError;
    VOResult{result_k}.name = 'positionError(m)';
    VOResult{result_k}.comment = '位置误差';
    VOResult{result_k}.frequency = combineFre ;
    % 计算最大相对误差        
    validLength = fix((length(VOsta)-1)*trueTraeFre/combineFre)+1;
    validLength = min(validLength,length(true_position));
    true_position_valid = true_position(:,1:validLength) ;
    text_error_xyz = GetErrorText( true_position_valid,VOstaError ) ;    
    VOResult{result_k}.text = text_error_xyz;
    
    if ~isempty(VOsta_trueRbb_error)
        result_k = result_k+1;
        VOResult{result_k}.data = VOsta_trueRbb_error;
        VOResult{result_k}.name = 'positionError_trueRbb(m)';
        VOResult{result_k}.comment = '无Rbb误差的位置误差';
        VOResult{result_k}.frequency = combineFre ;
        % 计算最大相对误差        
        validLength = fix((length(VOsta_trueRbb_error)-1)*trueTraeFre/combineFre)+1;
        true_position_valid = true_position(:,1:validLength) ;
        text_error_xyz = GetErrorText( true_position_valid,VOsta_trueRbb_error ) ;    
        VOResult{result_k}.text = text_error_xyz;
    end
    
    if ~isempty(VOsta_trueTbb_error)
        result_k = result_k+1;
        VOResult{result_k}.data = VOsta_trueTbb_error;
        VOResult{result_k}.name = 'positionError_trueTbb(m)';
        VOResult{result_k}.comment = '无Tbb误差的位置误差';
        VOResult{result_k}.frequency = combineFre ;
        % 计算最大相对误差        
        validLength = fix((length(VOsta_trueTbb_error)-1)*trueTraeFre/combineFre)+1;
        true_position_valid = true_position(:,1:validLength) ;
        text_error_xyz = GetErrorText( true_position_valid,VOsta_trueTbb_error ) ;    
        VOResult{result_k}.text = text_error_xyz;
    end
    
    if ~isempty(VOStaStepError)
        result_k = result_k+1;
        VOResult{result_k}.data = VOStaStepError;
        VOResult{result_k}.name = 'VOStaStepError(m)';
        VOResult{result_k}.comment = '视觉导航单步位置误差';
        VOResult{result_k}.frequency = combineFre ;
        VOResult{result_k}.subName = {'x(m)','y(m)','z(m)'};
        VOStaStepErrorMean = mean(VOStaStepError,2) ;
        text_mean_x = sprintf('x：%0.3e',VOStaStepErrorMean(1)) ;
        text_mean_y = sprintf('y：%0.3e',VOStaStepErrorMean(2)) ;
        text_mean_z = sprintf('z：%0.3e',VOStaStepErrorMean(3)) ;
        VOResult{result_k}.text = {'平均值',text_mean_x,text_mean_y,text_mean_z};
    end
    
    if ~isempty(VOStaStepError_A)
        result_k = result_k+1;
        VOResult{result_k}.data = VOStaStepError_A;
        VOResult{result_k}.name = 'VOStaStepError_A(m)';
        VOResult{result_k}.comment = '视觉导航单步位置误差A部分';
        VOResult{result_k}.frequency = combineFre ;
        VOResult{result_k}.subName = {'x(m)','y(m)','z(m)'};
        VOStaStepErrorAMean = mean(VOStaStepError_A,2) ;
        text_meanA_x = sprintf('x：%0.3e',VOStaStepErrorAMean(1)) ;
        text_meanA_y = sprintf('y：%0.3e',VOStaStepErrorAMean(2)) ;
        text_meanA_z = sprintf('z：%0.3e',VOStaStepErrorAMean(3)) ;
        VOResult{result_k}.text = {'平均值',text_meanA_x,text_meanA_y,text_meanA_z};
    end
    
    if ~isempty(VOStaStepError_B)
        result_k = result_k+1;
        VOResult{result_k}.data = VOStaStepError_B;
        VOResult{result_k}.name = 'VOStaStepError_B(m)';
        VOResult{result_k}.comment = '视觉导航单步位置误差B部分';
        VOResult{result_k}.frequency = combineFre ;
        VOResult{result_k}.subName = {'x(m)','y(m)','z(m)'};
        VOStaStepErrorAMean = mean(VOStaStepError_B,2) ;
        text_meanB_x = sprintf('x：%0.3e',VOStaStepErrorAMean(1)) ;
        text_meanB_y = sprintf('y：%0.3e',VOStaStepErrorAMean(2)) ;
        text_meanB_z = sprintf('z：%0.3e',VOStaStepErrorAMean(3)) ;
        VOResult{result_k}.text = {'平均值',text_meanB_x,text_meanB_y,text_meanB_z};
        %% 将VOCrbError 的3行分解
        L = length(VOCrbError) ;
        VOCrbError_x = zeros(3,L);
        VOCrbError_y = zeros(3,L);
        VOCrbError_z = zeros(3,L);
        for k=1:L
            VOCrbError_x(:,k) = VOCrbError(1,:,k);
            VOCrbError_y(:,k) = VOCrbError(2,:,k);
            VOCrbError_z(:,k) = VOCrbError(3,:,k);
        end
    
    
        result_k = result_k+1;
        VOResult{result_k}.data = VOCrbError_x;
        VOResult{result_k}.name = 'VOCrbError_x';
        VOResult{result_k}.comment = '视觉导航姿态矩阵误差第1行';
        VOResult{result_k}.frequency = combineFre ;
        VOResult{result_k}.subName = {'第1列','第2列','第3列'};
        VOResult{result_k}.addition = 'mean';

        result_k = result_k+1;
        VOResult{result_k}.data = VOCrbError_y;
        VOResult{result_k}.name = 'VOCrbError_y';
        VOResult{result_k}.comment = '视觉导航姿态矩阵误差第2行';
        VOResult{result_k}.frequency = combineFre ;
        VOResult{result_k}.subName = {'第1列','第2列','第3列'};
        VOResult{result_k}.addition = 'mean';

        result_k = result_k+1;
        VOResult{result_k}.data = VOCrbError_z;
        VOResult{result_k}.name = 'VOCrbError_z';
        VOResult{result_k}.comment = '视觉导航姿态矩阵误差第3行';
        VOResult{result_k}.frequency = combineFre ;
        VOResult{result_k}.subName = {'第1列','第2列','第3列'};
        VOResult{result_k}.addition = 'mean';
    end
    if ~isempty(VOCrcError)
    %% 将 VOCrcError 的3行分解
        L = length(VOCrcError) ;
        VOCrcError_x = zeros(3,L);
        VOCrcError_y = zeros(3,L);
        VOCrcError_z = zeros(3,L);
        for k=1:L
            VOCrcError_x(:,k) = VOCrcError(1,:,k);
            VOCrcError_y(:,k) = VOCrcError(2,:,k);
            VOCrcError_z(:,k) = VOCrcError(3,:,k);
        end

        result_k = result_k+1;
        VOResult{result_k}.data = VOCrcError_x;
        VOResult{result_k}.name = 'VOCrcError_x';
        VOResult{result_k}.comment = '失准角矩阵与单位阵差第1行';
        VOResult{result_k}.frequency = combineFre ;
        VOResult{result_k}.subName = {'第1列','第2列','第3列'};
        VOResult{result_k}.addition = 'mean';

        result_k = result_k+1;
        VOResult{result_k}.data = VOCrcError_y;
        VOResult{result_k}.name = 'VOCrcError_y';
        VOResult{result_k}.comment = '失准角矩阵与单位阵差第2行';
        VOResult{result_k}.frequency = combineFre ;
        VOResult{result_k}.subName = {'第1列','第2列','第3列'};
        VOResult{result_k}.addition = 'mean';

        result_k = result_k+1;
        VOResult{result_k}.data = VOCrcError_z;
        VOResult{result_k}.name = 'VOCrcError_z';
        VOResult{result_k}.comment = '失准角矩阵与单位阵差第3行';
        VOResult{result_k}.frequency = combineFre ;
        VOResult{result_k}.subName = {'第1列','第2列','第3列'};
        VOResult{result_k}.addition = 'mean';
    end
        
    if ~isempty(VOrcAngle)
        result_k = result_k+1;
        VOResult{result_k}.data = VOrcAngle;
        VOResult{result_k}.name = 'VOrcAngle';
        VOResult{result_k}.comment = '视觉导航平台失准角';
        VOResult{result_k}.frequency = combineFre ;
        VOResult{result_k}.subName = {'俯仰','横滚','航向'};
        VOResult{result_k}.addition = 'mean';
    end
    
    if ~isempty(VOStaStepError_Adefine)
        result_k = result_k+1;
        VOResult{result_k}.data = VOStaStepError_Adefine;
        VOResult{result_k}.name = 'VOStaStepError_Adefine(m)';
        VOResult{result_k}.comment = '视觉导航单步位置误差A部分(按定义)';
        VOResult{result_k}.frequency = combineFre ;
        VOResult{result_k}.subName = {'x(m)','y(m)','z(m)'};
        VOStaStepError_Adefine_Mean = mean(VOStaStepError_Adefine,2) ;
        text_meanA_x = sprintf('x：%0.3e',VOStaStepError_Adefine_Mean(1)) ;
        text_meanA_y = sprintf('y：%0.3e',VOStaStepError_Adefine_Mean(2)) ;
        text_meanA_z = sprintf('z：%0.3e',VOStaStepError_Adefine_Mean(3)) ;
        VOResult{result_k}.text = {'平均值',text_meanA_x,text_meanA_y,text_meanA_z};
    end
else
    VOResult = VOResult(1:4);
end
VOResult = VOResult(:,1:result_k);
if resultNum<result_k
   errordlg('出错，预设定的公共参数不全') 
end