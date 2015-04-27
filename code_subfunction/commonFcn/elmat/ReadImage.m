%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%           读取图片：适用 视景仿真/实验月球车/kitti 各种数据存储方式
% 使用分两步： 
%       1) 在读取第一图前设置图片路径： ReadImage('SetImagePath')
%       2) 读取第 N 图： [leftImage,rightImage] = ReadImage('GetImage',N)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [leftImage,rightImage,firstImageN,lastImageN] = ReadImage(command,n_image)


global projectDataPath leftPathName rightPathName leftSufffix rightSuffix leftPrefix rightPrefix imFormat 
switch command
    case 'SetImagePath'
    %% 设置图片读取路径
    if isempty(projectDataPath) % 独立运行此函数时
        inputDataPath_left = pwd; 
        inputDataPath_right = pwd; 
    else
        inputDataPath_left = [projectDataPath,'\相机1\定时采集'];   % 默认图像和IMU数据保存的文件夹
        inputDataPath_right= [projectDataPath,'\相机2\定时采集'];
        if ~isdir(inputDataPath_left)
            inputDataPath_left = uigetdir(projectDataPath,'选择 左 图像 路径');
            if inputDataPath_left==0
                return;
            end
        end
        if ~isdir(inputDataPath_right)
            inputDataPath_right = uigetdir(projectDataPath,'选择 右 图像 路径');
            if inputDataPath_right==0
                return;
            end
        end
    end
    %
    [leftFileName, leftPathName] = uigetfile({'*.bmp';'*.jpg';'*.png'},'选择左相机的任意一幅图，要求只有编号为数字',inputDataPath_left);
    if leftFileName==0
       return ;
    end
    [~, ~, imFormat] = fileparts(leftFileName) ;    % 图片格式：imFormat 
    [rightFileName, rightPathName] = uigetfile(['*',imFormat],'选择右相机的任意一幅图，要求只有编号为数字',inputDataPath_right);
    if rightFileName==0
       return ;
    end
    [leftPrefix,leftSufffix] = GetFileFix(leftFileName) ;
    [rightPrefix,rightSuffix] = GetFileFix(rightFileName) ;
    %% 计算图片个数
    if strcmp(leftPathName,rightPathName)==1
        % 左右在一个文件夹
        allImageFile = ls([leftPathName,['*',imFormat]]);  % 所有左右相机图片的文件名
        imNum = fix(size(allImageFile,1)/2);
    else
        leftImageFile = ls([leftPathName,leftPrefix,['*',imFormat]]);  % 所有左相机图片的文件名
        rightImageFile = ls([rightPathName,rightPrefix,['*',imFormat]]);
        imNum = min(size(leftImageFile,1),size(rightImageFile,1));   % 时刻数
    end
    answer = inputdlg('起始图片序号是0还是1','起始图片序号是0还是1',1,{'1'}) ;
    firstImageN = str2double(answer{1}) ;
    lastImageN = imNum-1+firstImageN ;    
    disp(['时刻数： ',num2str(imNum)])   

    leftImage=[];
    rightImage=[];
    case 'GetImage'
    %% 读取图片的内容
    leftImage = imread([leftPathName,getImageName(leftPrefix,n_image,leftSufffix),imFormat]);   
    rightImage = imread([rightPathName,getImageName(rightPrefix,n_image,rightSuffix),imFormat ]);
end

function imName = getImageName(Prefix,i,Sufffix)
if ~isempty(Prefix) && ~isempty(Sufffix)
    imName = [Prefix,num2str(i),Sufffix] ;  % 视觉仿真的图片从1开始命名
else
    imName = num2str(i,'%010d');          % kitti的图片格式  从0开始命名
end

function [prefix,suffix] = GetFileFix(filename)
% 搜索所有非数字部分
if isNumStr(filename(1))
   disp('没有前缀'); 
   prefixNum = 0;
   prefix=[];
else
    prefixNum = 1 ; % 记录非数字字符的个数
    for i=2:length(filename)
       if ~isNumStr(filename(i))  && ~isNumStr(filename(i-1))
           prefixNum = prefixNum+1 ;    % 找到一个字符  
       else
            break;
       end
    end
    prefix = filename(1:prefixNum); % 前缀
end

for i=prefixNum+1:length(filename)
   if ~isNumStr(filename(i)) 
       break;
   end
end
suffixNum = i;
for i_last=prefixNum+1:length(filename)
   if strcmp(filename(i_last),'.')
       break;
   end
end
if(suffixNum==i_last)
   suffix = [];     % 后缀
else
    suffix = filename(suffixNum:i_last-1);
end


function isNum = isNumStr(character)
% 
if strcmp(character,'i')
   isNum = 0; 
   return;
end
if isnan( str2double(character) )
    isNum = 0; 
else
    isNum = 1; 
end
