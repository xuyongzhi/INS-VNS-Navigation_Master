% check X，R，T
%   X=[ dAngle,dV,dR,gyroDrift,accDrift ]

% savePath = 'E:\惯性视觉导航\程序\综合程序\data\组合算法测试\向前40m视觉无误差\原师姐版\XRT';

function figh = CheckResult(check,integMethod,resultPath)
if ~exist('check','var')
    check = evalin('base', 'check') ;
end
if ~exist('integMethod','var')
    integMethod = '';
end
if ~exist('resultPath','var')
    resultPath = [pwd,'\checkResult'];
else
    resultPath = [resultPath,'\checkResult'];
end

if isdir(resultPath)
    delete([resultPath,'\*']);
else
   mkdir(resultPath); 
end

 %projectName = 'augment_dRdT';
projectName = 'simple_dRdT';

% X_correct = eval(['X_correct_',projectName]);
% X_pre_error = eval(['X_pre_error_',projectName]);
% T_INS = eval(['T_INS_',projectName]);
% T_VNS = eval(['T_VNS_',projectName]);
% R_INS = eval(['R_INS_',projectName]);
% R_VNS = eval(['R_VNS_',projectName]);

X_correct = check.X_correct;
X_pre_error = check.X_pre_error;
T_INS = check.T_INS;
T_VNS = check.T_VNS;
R_INS = check.R_INS;
R_VNS = check.R_VNS;
newInformation = check.newInformation ;

prompt = {'xLook:','RLook:','TLook:','newInfLook:'};
dlg_title = '选择需要绘制的图形';
num_lines = 1;
def = {'1','0','0','1'};
answer = inputdlg(prompt,dlg_title,num_lines,def)

xLook = str2double(answer{1}) ;
RLook = str2double(answer{2}) ;
TLook = str2double(answer{3}) ;
newInfLook = str2double(answer{4}) ;

figh = zeros(1,10);
figNum = 0;

XName = {'angle\_x','angle\_y','angle\_z','dv\_x','dv\_y','dv\_z','dr\_x','dr\_y','dr\_z','drift\_gyro\_x','drift\_gyro\_y','drift\_gyro\_z','drift\_acc\_x','drift\_acc\_y','drift\_acc\_z'};
XName_save = {'angle_x','angle_y','angle_z','dv_x','dv_y','dv_z','dr_x','dr_y','dr_z','drift_gyro_x','drift_gyro_y','drift_gyro_z','drift_acc_x','drift_acc_y','drift_acc_z'};
if xLook==1
    % X修正效果
    for i=1:15
        X_correcti=X_correct(i,:);
        X_pre_errori=X_pre_error(i,:);
        N = length(X_correcti);
        figNum = figNum+1 ;
        figh(figNum) = figure('name',['一步预测误差和状态修正量',XName_save{i}],'position',[700   294   672   504]) ;
        plot(1:N,[X_correcti;X_pre_errori],'LineWidth',2)
        title(XName{i})
        hold on
        line([0 N],[0 0],'Color','black')
        legend({'X\_correct','X\_pre\_error'});
        
        saveas(gcf,[resultPath,'\',XName_save{i}],'fig')
        saveas(gcf,[resultPath,'\',XName_save{i}],'emf')
        %close(gcf)
    end
end

T_true = [0 -0.3 0];
if TLook ==1
    % 量测信息：组导航信息源
    % T
    for i=1:3
        T_INSi=T_INS(i,:);
        T_VNSi=T_VNS(i,:);
        N = length(T_VNSi);
        figNum = figNum+1 ;
        figh(figNum) = figure('name',['T_INS-',num2str(i)],'position',[700   294   672   504])  ;      
        plot(1:N,[T_INSi;T_VNSi],'LineWidth',2)
        title(['T\_INS-',num2str(i)])
        legend({'T\_INSi','T\_VNSi'});
        hold on
        line([0 N],[T_true(i) T_true(i)],'Color','black')
        
        saveas(gcf,[resultPath,'\T-',num2str(i)],'emf')
        saveas(gcf,[resultPath,'\T-',num2str(i)],'fig')
        %close(gcf)
    end
end


if RLook==1
    % R
    for i=1:3
        Angle_INS = zeros(3,N);
        Angle_VNS = zeros(3,N);
        for k=1:N
            Angle_INS(:,k)=GetAttitude(R_INS(:,:,k));
            Angle_VNS(:,k)=GetAttitude(R_VNS(:,:,k));
        end
        Angle_INSi=Angle_INS(i,:);
        Angle_VNSi=Angle_VNS(i,:);
        N = length(Angle_INSi);
        figNum = figNum+1 ;
        figh(figNum) = figure('name',['R_INS-',num2str(i)],'position',[700   294   672   504]) ; 
        plot(1:N,[Angle_INSi;Angle_VNSi],'LineWidth',2)
        title(['R\_INS-',num2str(i)])
        legend({'Angle\_INSi','Angle\_VNSi'});
        hold on
        line([0 N],[-0 -0],'Color','black')
        
         saveas(gcf,[resultPath,'\R-',num2str(i)],'emf')
         saveas(gcf,[resultPath,'\R-',num2str(i)],'fig')
         %close(gcf)
    end
end

newInfName = {'newInf\_dRx','newInf\_dRy','newInf\_dRz','newInf\_dTx','newInf\_dTy','newInf\_dTz'};
newInfName_save = {'newInf_dRx','newInf_dRy','newInf_dRz','newInf_dTx','newInf_dTy','newInf_dTz'};
if newInfLook==1
    % 新息
    for i=1:6
        newInformationi=newInformation(i,:);
        N = length(newInformationi);
        figNum = figNum+1 ;
        figh(figNum) = figure('name',['新息-',newInfName_save{i}],'position',[700   294   672   504]) ;
        plot(1:N,newInformationi,'LineWidth',2)
        title(newInfName{i})
        hold on
        line([0 N],[0 0],'Color','black')
        
        saveas(gcf,[resultPath,'\',newInfName_save{i}],'fig')
        saveas(gcf,[resultPath,'\',newInfName_save{i}],'emf')
        %close(gcf)
    end
end

figh = figh(1:figNum) ;

%% 将前几步的滤波过程记录下来
N = 3 ;
fileID = fopen([resultPath,'\',integMethod,'_FilterProcess.txt'],'w') ;
fprintf(fileID,'\n\t\t前%d步滤波过程记录',N);
for k=1:N
    fprintf(fileID,'\n第%d步：',k);
    fprintf(fileID,'\n\t一步预测误差：%s',num2str(X_pre_error(k,:)));
    fprintf(fileID,'\n\t滤波状态修正：%s',num2str(X_correct(k,:)));
    fprintf(fileID,'\n\t新息：%s',num2str(newInformation(k,:)));
    
end
fclose(fileID);