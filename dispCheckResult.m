% 显示滤波检查结果
%  调用 CheckResult
global navResultPath
if isempty(navResultPath)
    navResultPath = [pwd,'\data'];
end

if exist('augment_dRdT_check','var')
   button = questdlg('是否显示augment_dRdT_check'); 
   if strcmp(button,'Yes')
       CheckResult(augment_dRdT_check,'augment_dRdT',navResultPath) ;
   end
   disp('augment_dRdT_check OK')
end
if exist('simple_dRdT_check','var')
   button = questdlg('是否显示simple_dRdT_check'); 
   if strcmp(button,'Yes')
       CheckResult(augment_dRdT_check,'simple_dRdT',navResultPath) ;
   end
   disp('simple_dRdT_check OK')
end

disp('结束')
