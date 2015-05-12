% buaa xyz 2014.1.16

% 保存INS_VNS的组合导航结果为ResultDisplay模块特定格式（参考相关说明文档）

function VOResult = saveVOResult( isKnowTrue,frequency_VO,VOsta,VOpos,VOvel,matchedNum,...
    VOposError,VOvelError, VOstaError,combineFre,trueTraeFre,true_position )
resultNum = 7;
VOResult = cell(1,resultNum);

% 有4个共同的成员
for j=1:resultNum
    VOResult{j}.dataFlag = 'xyz result display format';
    VOResult{j}.frequency = frequency_VO ;
    VOResult{j}.project = 'VO';
    VOResult{j}.subName = {'x','y','z'};
end

VOResult{1}.data = VOsta;
VOResult{1}.name = 'position(m)';
VOResult{1}.comment = '位置';

VOResult{2}.data = VOpos*180/pi ;   % 转为角度单位
VOResult{2}.name = 'attitude(°)';
VOResult{2}.comment = '姿态';
VOResult{2}.subName = {'俯仰','横滚','航向'};

VOResult{3}.data = VOvel;
VOResult{3}.name = 'velocity(m/s)';
VOResult{3}.comment = '速度';

VOResult{4}.data = matchedNum;
VOResult{4}.name = 'matchedNum';
VOResult{4}.comment = '特征点个数';
VOResult{4}.subName = [];

if  isKnowTrue==1
    VOResult{5}.data = VOposError*180/pi*3600;
    VOResult{5}.name = 'attitudeError('')';
    VOResult{5}.comment = '姿态误差';
    VOResult{5}.subName = {'俯仰','横滚','航向'};
    VOResult{5}.frequency = combineFre ;

    VOResult{6}.data = VOvelError;
    VOResult{6}.name = 'velocityError(m/ｓ)';
    VOResult{6}.comment = '速度误差';
    VOResult{6}.frequency = combineFre ;
    
    VOResult{7}.data = VOstaError;
    VOResult{7}.name = 'positionError(m)';
    VOResult{7}.comment = '位置误差';
    VOResult{7}.frequency = combineFre ;
    % 计算最大相对误差        
    validLength = fix(length(VOsta)*trueTraeFre/combineFre);
    true_position_valid = true_position(:,1:validLength) ;
    text_error_xyz = GetErrorText( true_position_valid,VOstaError ) ;
    
    VOResult{7}.text = text_error_xyz;
else
    VOResult = VOResult(1:4);
end