%% 保存视觉估计误差结果
%  buaaxyz 2014 10 7

function INS_VNS_NavResult2 = saveResult_SINSerror_dMove_new2(integFre,projectName,vnsTbbDriftError,vnsRbbDriftError,vnsTbbDrift_true,vnsRbbDrift_true)

% 存储为特定格式：每个变量一个细胞，包含成员：data，name,comment 和 dataFlag,frequency,project,subName
resultNum = 4;
INS_VNS_NavResult2 = cell(1,resultNum);

% 有4个共同的成员
for j=1:resultNum
    INS_VNS_NavResult2{j}.dataFlag = 'xyz result display format';
    INS_VNS_NavResult2{j}.frequency = integFre ;
    INS_VNS_NavResult2{j}.project = projectName ;
    INS_VNS_NavResult2{j}.subName = {'x(m)','y(m)','z(m)'};
end

resultN = 1;
INS_VNS_NavResult2{resultN}.data = vnsTbbDriftError ;
INS_VNS_NavResult2{resultN}.name = 'vnsTbbDriftError(m)';
INS_VNS_NavResult2{resultN}.comment = '视觉平移漂移估计误差';
INS_VNS_NavResult2{resultN}.subName = {'bx(m)','by(m)','bz(m)'};

resultN = resultN+1;
INS_VNS_NavResult2{resultN}.data = vnsRbbDriftError*180/pi ;
INS_VNS_NavResult2{resultN}.name = 'vnsRbbDriftError(°)';
INS_VNS_NavResult2{resultN}.comment = '视觉旋转漂移估计误差';
INS_VNS_NavResult2{resultN}.subName = {'x(°)','y(°)','z(°)'};

resultN = resultN+1;
INS_VNS_NavResult2{resultN}.data = vnsTbbDrift_true ;
INS_VNS_NavResult2{resultN}.name = 'vnsTbbDrift_true(m)';
INS_VNS_NavResult2{resultN}.comment = '真实视觉平移漂移';
INS_VNS_NavResult2{resultN}.subName = {'bx(m)','by(m)','bz(m)'};

resultN = resultN+1;
INS_VNS_NavResult2{resultN}.data = vnsRbbDrift_true*180/pi ;
INS_VNS_NavResult2{resultN}.name = 'vnsRbbDrift_true(°)';
INS_VNS_NavResult2{resultN}.comment = '真实视觉旋转漂移';
INS_VNS_NavResult2{resultN}.subName = {'x(°)','y(°)','z(°)'};