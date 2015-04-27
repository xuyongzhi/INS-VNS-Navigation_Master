%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%                             xyz
%                           2014.3.7
%                          记录输入数据到txt文件
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function RecodeInput (fid,visualInputData,imuInputData,trueTrace)

fprintf(fid,'%s','       实验笔记\n');

%% 记录输入参数
% 轨迹发生器
if isfield(trueTrace,'traceRecord')
    fprintf(fid,'采用轨迹发生器生成轨迹\n%s\n\n',trueTrace.traceRecord) ;
end
if isfield(trueTrace,'InitialPositionError')
    str1 = sprintf('%0.4f  ',trueTrace.InitialPositionError) ;
    str2 = sprintf('%0.4f  ',trueTrace.InitialAttitudeError) ;
    str3 = sprintf('%0.4f  ',trueTrace.InitialAttitudeError * 180/pi*3600) ;
    fprintf(fid,'初始位置误差：%s m\n初始姿态误差：%s rad\n初始姿态误差：%s ″\n\n',str1,str2,str3);
else
    fprintf(fid,'无初始位置、姿态误差\n\n');
end
% 视觉信息
if ~isempty(visualInputData)
    fprintf(fid,'视觉信息频率：%0.3f\n',visualInputData.frequency);
    calibData=visualInputData.calibData ;
    
    % 左相机相对车体位置
    if isfield(calibData,'Tcb_c')
        str_Tcb_c = sprintf('%0.3g ',calibData.Tcb_c);
        fprintf(fid,'左相机相对车体位置(Tcb_c)：%s m',str_Tcb_c);
        if isfield(calibData,'Tcb_c_error') && isfield(calibData,'isEnableCalibError') &&    calibData.isEnableCalibError == 1  
            str_Tcb_c_error = sprintf('%0.3g ',calibData.Tcb_c_error );
            fprintf(fid,'\t  误差：%s m\n',str_Tcb_c_error);
        else
            fprintf(fid,'\n');
        end
    end
    % 左相机相对车体安装角     
    if isfield(calibData,'cameraSettingAngle')
        str_cameraSettingAngle = sprintf('%0.3g ',calibData.cameraSettingAngle*180/pi);
        fprintf(fid,'左相机相对车体安装角：%s °',str_cameraSettingAngle);
        if isfield(calibData,'cameraSettingAngle_error') && isfield(calibData,'isEnableCalibError') &&    calibData.isEnableCalibError == 1  
            str_cameraSettingAngle_error = sprintf('%0.3g ',calibData.cameraSettingAngle_error*180/pi);
            fprintf(fid,'\t  误差：%s °\n',str_cameraSettingAngle_error);
        else
            fprintf(fid,'\n');
        end
    end
    % 右相机相对左相机位置
    str_T = sprintf('%0.3g ',calibData.T);
    fprintf(fid,'真实右相机相对左相机位置(T)：%s mm',str_T);
    if isfield(calibData,'T_error') && isfield(calibData,'isEnableCalibError') &&    calibData.isEnableCalibError == 1  
        T_error=calibData.T_error;
        str_T_error = sprintf('%0.3g ',T_error);
        fprintf(fid,'\t 误差：%s mm\n',str_T_error);
    else
        fprintf(fid,'\n');
    end
    % 右相机相对左相机安装角
    str_om = sprintf('%0.3g ',calibData.om*180/pi);
    fprintf(fid,'真实右相机相对左相机安装角(om)：%s °',str_om);
    if isfield(calibData,'om_error') && isfield(calibData,'isEnableCalibError') &&    calibData.isEnableCalibError == 1  
        om_error=calibData.om_error;
        str_om_error = sprintf('%0.3g ',om_error*180/pi);
        fprintf(fid,'\t 误差：%s °\n',str_om_error);
    else
        fprintf(fid,'\n');
    end
    % 焦距 calibData.fc_left
    str_fc_left = sprintf('%0.3g ',calibData.fc_left);
    fprintf(fid,'真实左焦距(fc_left)：%s 像素',str_fc_left);
    if isfield(calibData,'fc_left_error') && isfield(calibData,'isEnableCalibError') &&    calibData.isEnableCalibError == 1  
        fc_left_error=calibData.fc_left_error;
        str_fc_left_error = sprintf('%0.3g ',fc_left_error);
        fprintf(fid,'\t 误差：%s 像素\n',str_fc_left_error);
    else
        fprintf(fid,'\n');
    end
    
    % 没有误差
    if ~isfield(calibData,'isEnableCalibError') ||   calibData.isEnableCalibError == 0
        fprintf(fid,'\n\t 双目视觉仿真系统无误差\n');
    end
     
    if isfield(visualInputData,'RTError')
        textStr =  '直接仿真生成RT，以下为RT生成所添加的噪声：\n';
      %  str = num2str(visualInputData.RTError.TbbErrorMean);
        str = sprintf('%0.3g ',visualInputData.RTError.TbbErrorMean);
        textStr = [textStr  'TbbError均值：'  str  ' (m)\n'];
 %       str = num2str(visualInputData.RTError.TbbErrorStd);
        str = sprintf('%0.3g ',visualInputData.RTError.TbbErrorStd);
        textStr = [textStr   'TbbError标准差：'   str   '(m)\n'];

    %    str1 = num2str(visualInputData.RTError.AngleErrorMean);
        str1 = sprintf('%0.3g ',visualInputData.RTError.AngleErrorMean);
   %     str2 = num2str(visualInputData.RTError.AngleErrorMean*180/pi*3600);
        str2 = sprintf('%0.3g ',visualInputData.RTError.AngleErrorMean*180/pi*3600);
        textStr = [textStr   'AngleError均值：'   str1 ,'(rad)  ', str2,  '(″)\n'];
     %   str1 = num2str(visualInputData.RTError.AngleErrorStd);
        str1 = sprintf('%0.3g ',visualInputData.RTError.AngleErrorStd);
   %     str2 = num2str(visualInputData.RTError.AngleErrorStd*180/pi*3600);
        str2 = sprintf('%0.3g ',visualInputData.RTError.AngleErrorStd*180/pi*3600);
        textStr = [textStr  'AngleError标准差：'   str1 ,'(rad)  ', str2,  '(″)\n'];

        fprintf(fid,textStr);
    end
end
% 惯导信息
planet = trueTrace.planet ;
if strcmp(planet,'m')
    moonConst = getMoonConst;   % 得到月球常数
    gp = moonConst.g0 ;     % 用于导航解算
    wip = moonConst.wim ;
    fprintf(fid,'\n惯导信息(月球)\n');
else
    earthConst = getEarthConst;   % 得到地球常数
    gp = earthConst.g0 ;     % 用于导航解算
    wip = earthConst.wie ;
    fprintf(fid,'\n惯导信息（地球）\n');
end
if ~isempty(imuInputData)
    fprintf(fid,'惯导信息频率：%0.1f\n',imuInputData.frequency);
    if isfield(imuInputData,'pa')    
        pa = imuInputData.pa/(gp*1e-6);
        fprintf(fid,'加计常值偏置：(%g,%g,%g) (ug)\n',pa(1),pa(2),pa(3));
        na = imuInputData.na/(gp*1e-6);
        fprintf(fid,'加计随机漂移：(%g,%g,%g) (ug)\n',na(1),na(2),na(3));
        pg = imuInputData.pg*180/pi*3600;
        fprintf(fid,'陀螺常值偏置：(%g,%g,%g) (°/h)\n',pg(1),pg(2),pg(3));
        ng = imuInputData.ng*180/pi*3600;
        fprintf(fid,'陀螺随机漂移：(%g,%g,%g) (°/h)\n',ng(1),ng(2),ng(3));
    end
end
