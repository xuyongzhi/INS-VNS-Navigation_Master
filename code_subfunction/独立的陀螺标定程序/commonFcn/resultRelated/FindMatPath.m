%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 开始日期：2013.12.2
% 作者：xyz
% 功能：寻找.mat格式文件的路径，根据文件的关键识别字符keystr
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function matFindResultPath = FindMatPath(findPath,keyStr)

% 输入：待寻找的文件夹的路径 findPath，待寻找文件名的关键识别字符 keyStr
% 输出： 待寻找文件的完整文件路径 resultPath，不存在则返回0
matFindResultPath=0;    % 输出预置0

allFiles = dir([findPath,'\*.mat']);
filenum = size(allFiles);
findSuccNum = 0;    %成功找到的个数
for i=1:filenum
   fileName =  allFiles(i).name;
   iFindResult = strfind(fileName,keyStr);  % 从fileName中查找字符串keyStr
   if(~isempty(iFindResult))    % 找到了
      findSuccNum = findSuccNum+1; 
      if findSuccNum==1
           matFindResultPath = [findPath,'\',fileName];
       else
           errordlg({'找到多个符合的文件！',['取第一个：',matFindResultPath]});
       end
   end
end
if(findSuccNum==0)
    errordlg('没找到符合的文件！');
    matFindResultPath=0;
end
