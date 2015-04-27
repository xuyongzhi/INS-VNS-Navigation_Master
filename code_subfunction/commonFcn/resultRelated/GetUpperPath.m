% 作者：xyz
% 日期：2013.12.23
% 功能：得到路径的上一层路径
% 输入：路径字符串
% 输出：上一层路径和最后的文件夹名

function [upperPath,curName] = GetUpperPath(path)
% 找到最后一个'\'的序号
for k=1:length(path)
    if strcmp(path(k),'\')
       curNum = k; 
    end    
end
upperPath = path(1:curNum-1);
curName = path(curNum+1:length(path));