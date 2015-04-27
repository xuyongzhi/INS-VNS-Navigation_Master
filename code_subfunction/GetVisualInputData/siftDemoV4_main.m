%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 开始日期：2013.12.4
%       2014.5.17 日改 特征点显示等
% 作者：xyz
% 功能：sift特征点提取入口函数
%   输入：	图片所保存的文件地址，左右图片各提前一次地址
%          图片的命名规则
%   输出 visualInputData
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 通过对话框提取一幅图的路径和命名，要求图片的命名规则为：前面是图片名称前缀（非数字），后面是图片编号
% 左右相机的图片分别提取一次
function visualInputData = siftDemoV4_main()
%% visualInputData
% [1*N ] cell 数组， N为时刻数，每个cell为一个时刻前后4副图的匹配成功特征点，1个cell visualInputData{i}. 中包含4个成员：
% leftLocCurrent：左相机当前图匹配成功的特征点，[2*n]，n为该帧前后4副图匹配成功的特征点个数。
% rightLocCurrent：右相机当前图匹配成功的特征点
% leftLocNext：左相机下时刻图匹配成功的特征点
% rightLocNext：右相机下时刻图匹配成功的特征点
% matchedNum ：匹配成功特征点个数 double
% aveFeatureNum ：该时刻前后4副图特征点个数的平均值（未做匹配时） double
%dbstop in MatchTwoImage

%% 获取图像路径和命名

% 得到输入数据的路径 inputDataPath_left inputDataPath_right
global projectDataPath leftPathName imFormat doFigureNum
doFigureNum = 5;        % 绘图时刻数，为-1则每个时刻绘图
maxPlotFeatureN = 20;  % 为0则不限  只正对匹配图，独立图将所有特征点画出
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
    allImageFile = ls([leftPathName,['*',imFormat]]);  % 所有左右相机图片的文件名
    imNum = fix(size(allImageFile,1)/2);
else
    leftImageFile = ls([leftPathName,leftPrefix,['*',imFormat]]);  % 所有左相机图片的文件名
    rightImageFile = ls([rightPathName,rightPrefix,['*',imFormat]]);
    imNum = min(size(leftImageFile,1),size(rightImageFile,1));   % 时刻数
end
disp(['时刻数： ',num2str(imNum)])
%%  记录参数 先
% answer = inputdlg({['图片的采集频率 共',num2str(imNum),'时刻'],'相机安装角(俯仰,横滚,航向)°   .'},'图片采集参数',1,{'1','0 0 0'});
answer = inputdlg({['图片的采集频率 共',num2str(imNum),'时刻  .']},'图片采集频率',1,{'1'});
frequency = str2double(answer{1});
% cameraSettingAngle = sscanf(answer{2},'%f')*pi/180;  

%% 匹配结果图片存储路径

featureFigureSavePath = [leftPathName,'特征点匹配图'];
if isdir(featureFigureSavePath)
    delete([featureFigureSavePath,'\*']);
else
    mkdir(featureFigureSavePath); 
end
%% 结果变量
leftLocCurrent = cell(1,imNum-1);  % 存储左相机当前图匹配成功的特征点，一个细胞一个时刻
rightLocCurrent = cell(1,imNum-1);
leftLocNext = cell(1,imNum-1);
rightLocNext = cell(1,imNum-1);
matchedNum = zeros(1,imNum-1);
aveFeatureNum = zeros(1,imNum-1);
    % 每个细胞中一个结构体，注意两个坐标合到一行中

waitbar_h = waitbar(0,{'开始特征点提取与匹配...'});
tic
if exist('siftDataTemp.mat','file')
   siftDataTemp = importdata('siftDataTemp.mat') ;
   endi = siftDataTemp.endi;
   button = questdlg(sprintf('上次存储的临时文件中已完成%d个，是否 重新提取特征点？',endi));
   if strcmp(button,'No')
       load('siftDataTemp.mat') ;
       starti = endi+1;
       fprintf('从%d对图开始提取特征点',starti);
   else
       starti = 1;
   end
else
    starti=1;
end
for i=starti:imNum-1
        
    disp(['第 ',num2str(i),' / ',num2str(imNum-1),' 个时刻：',sprintf('%d',toc),'sec']);
    
   %% 提取一个时刻匹配成功的特征点：针对当前帧与下一帧的四副图片 
   %% 仅在i==1时需要提前当前
    imageCurrent = [];
    if i==1
        if ~exist([leftPathName,getImageName(leftPrefix,i,leftSufffix),imFormat],'file')
            disp(['缺少图片：',leftPathName,getImageName(leftPrefix,i,leftSufffix),imFormat])
        else
            leftImageCurrent = imread([leftPathName,getImageName(leftPrefix,i,leftSufffix),imFormat]);   
            disp([leftPathName,getImageName(leftPrefix,i,leftSufffix),imFormat])
            rightImageCurrent = imread([rightPathName,getImageName(rightPrefix,i,rightSuffix),imFormat ]);
            disp([rightPathName,getImageName(rightPrefix,i,rightSuffix),imFormat ])
            % 视景仿真图片的分辨率修改问题
            if i==1 
                isCut = 0 ;
              %  if length(leftImageCurrent)>1392
                    isCutStr = questdlg('是否裁剪','图片裁剪','是','否','是') ;
                    if strcmp(isCutStr,'是')
                        isCut = 1 ;
                        resolutionStr=inputdlg({'裁剪目标分辨率（水平*垂直）'},'裁剪',1,{'1024 1024'});
                        resolution = sscanf(resolutionStr{1},'%f');
                    else
                        resolution = [size(leftImageCurrent,2),size(leftImageCurrent,1)];
                    end
          %      end
            end
            if isCut==1
                leftImageCurrent = leftImageCurrent(1:resolution(2),1:resolution(1),:); % 注意水平方向是列
                rightImageCurrent = rightImageCurrent(1:resolution(2),1:resolution(1),:);
            end
            % % 彩色=>黑白
            if ndims(leftImageCurrent)==3 % 彩色图片            
                leftImageCurrent = rgb2gray(leftImageCurrent);
                rightImageCurrent = rgb2gray(rightImageCurrent);
            end
            % 增强对比度        
            leftImageCurrent = imadjust(leftImageCurrent);
            rightImageCurrent = imadjust(rightImageCurrent);
            % 直方图均衡化
    %     leftImageCurrent = histeq(leftImageCurrent);
    %     rightImageCurrent = histeq(rightImageCurrent);
    %       保存处理后的图片

            if isdir([leftPathName,'处理后的图片'])
                delete([leftPathName,'处理后的图片','\*']);
            else
                mkdir([leftPathName,'处理后的图片']) ;
            end
            if isdir([rightPathName,'处理后的图片'])
                delete([rightPathName,'处理后的图片','\*']);
            else
                mkdir([rightPathName,'处理后的图片']) ;
            end
            imwrite(leftImageCurrent,[leftPathName,'处理后的图片','\leftImage',num2str(i),imFormat]);
            imwrite(rightImageCurrent,[rightPathName,'处理后的图片','\rightImage',num2str(i),imFormat]);

            imageCurrent.leftImageCurrent = leftImageCurrent ;
            imageCurrent.rightImageCurrent = rightImageCurrent ;
        end
    else
        imageCurrent = NextTwoMatchResult ; % 取上次存储的匹配结果
        %　仍然载入图片用于绘制特征点分布图
        imageCurrent.leftImageCurrent = leftImageNext ;     % 当前的图片即上一时刻留下的“下一时刻图片”
      	imageCurrent.rightImageCurrent = rightImageNext ;
    end
    %% 提下一时刻的图片
    if ~exist([leftPathName,getImageName(leftPrefix,i+1,leftSufffix),imFormat],'file')
    	disp(['缺少图片：',leftPathName,getImageName(leftPrefix,i+1,leftSufffix),imFormat])
    else
        leftImageNext = imread([leftPathName,getImageName(leftPrefix,i+1,leftSufffix),imFormat]);
        disp([leftPathName,getImageName(leftPrefix,i+1,leftSufffix),imFormat])
        rightImageNext = imread([rightPathName,getImageName(rightPrefix,i+1,rightSuffix),imFormat]);
        disp([rightPathName,getImageName(rightPrefix,i+1,rightSuffix),imFormat])
        if isCut==1
            leftImageNext = leftImageNext(1:resolution(2),1:resolution(1),:);
            rightImageNext = rightImageNext(1:resolution(2),1:resolution(1),:);
        end
        if ndims(leftImageNext)==3 % 彩色图片
            % 彩色=>黑白
            leftImageNext = rgb2gray(leftImageNext);
            rightImageNext = rgb2gray(rightImageNext);
        end
       % 增强对比度
        leftImageNext = imadjust(leftImageNext);
        rightImageNext = imadjust(rightImageNext);
        % 直方图均衡化
    %     leftImageNext = histeq(leftImageNext);
    %     rightImageNext = histeq(rightImageNext);
        %保存处理后的图片
        imwrite(leftImageNext,[leftPathName,'处理后的图片','\leftImage',num2str(i+1),imFormat]);
        imwrite(rightImageNext,[rightPathName,'处理后的图片','\rightImage',num2str(i+1),imFormat]);
        %% 提取匹配特征点

        imageNext.leftImageNext = leftImageNext;
        imageNext.rightImageNext = rightImageNext;
        
        [leftLocCurrent{i},rightLocCurrent{i},leftLocNext{i},rightLocNext{i},NextTwoMatchResult,matchedNum(i),aveFeatureNum(i)] = MatchFourImage(imageCurrent,imageNext,i,featureFigureSavePath,maxPlotFeatureN);
    end
   if mod(i,ceil((imNum-1)/10)==0)
        onePointTime = toc/(i+1);
        waitbarStr = sprintf('已完成第%d个时刻，已用时%0.1fsec,预计还需%0.1fsec',i,toc,onePointTime*(imNum-1-i));
        waitbar(i/(imNum-1),waitbar_h,waitbarStr);
   end
   if mod(i,1)==0
       endi = i;    % 记录当前完成的特征点提取个数，下次从 endi+1 开始提取
      save  siftDataTemp leftLocCurrent rightLocCurrent leftLocNext rightLocNext matchedNum aveFeatureNum frequency  NextTwoMatchResult leftImageNext rightImageNext isCut resolution endi
   end
end
close(waitbar_h);
% 

% 存储匹配点
visualInputData.leftLocCurrent = leftLocCurrent;
visualInputData.rightLocCurrent = rightLocCurrent;
visualInputData.leftLocNext = leftLocNext;
visualInputData.rightLocNext = rightLocNext;
visualInputData.matchedNum = matchedNum;
visualInputData.aveFeatureNum = aveFeatureNum;
visualInputData.frequency = frequency;

save([pwd,'\visualInputData.mat'],'visualInputData')  
save([leftPathName,'\visualInputData.mat'],'visualInputData')  
% 选择是否加入相机标定参数。添加：如果是真实实验采集，则加载，如果是视景仿真采集，则计算。
% if ~isfield(visualInputData,'calibData')
%     calibData = loadCalibData(resolution);
%     if ~isempty(calibData)
%         visualInputData.calibData = calibData;
%     end
% end
save([pwd,'\visualInputData.mat'],'visualInputData')  
save([leftPathName,'\visualInputData.mat'],'visualInputData')  
% save([pwd,'\siftMatchResult\visualInputData.mat'],'visualInputData')  
assignin('base','visualInputData',visualInputData)
sprintf('图片的特征点提取结束，已保存到  %s 、 当前目录 和base空间',leftPathName)

function imName = getImageName(Prefix,i,Sufffix)
if ~isempty(Prefix) && ~isempty(Sufffix)
    imName = [Prefix,num2str(i),Sufffix] ;  % 视觉仿真的图片从1开始命名
else
    imName = num2str(i-1,'%010d');          % kitti的图片从0开始命名
end

function calibData = loadCalibData(reslution)
% 选择是否加入相机标定参数。添加：如果是真实实验采集，则加载，如果是视景仿真采集，则计算。
global projectDataPath
button =  questdlg('根据图片获取的方法选择','添加标定数据','视景仿真：计算标定参数','真实实验：载入标定参数文件','不添加','视景仿真：计算标定参数') ;
if strcmp(button,'视景仿真：计算标定参数')
    calibData = GetCalibData(reslution) ;
end
if strcmp(button,'真实实验：载入标定参数文件')
    if isempty(projectDataPath) % 独立运行此函数时
        calibDataPath = pwd; 
    else
        calibDataPath = [GetUpperPath(projectDataPath),'\相机标定数据'];   % 默认相机标定数据路径
    end
    [cameraCalibName,cameraCalibPath] = uigetfile('.mat','选择相机标定数据',[calibDataPath,'\*.mat']);
    calibData = importdata([cameraCalibPath,cameraCalibName]); 
end
if strcmp(button,'不添加')
    calibData = [];
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

function [leftLocCurrent,rightLocCurrent,leftLocNext,rightLocNext,NextTwoMatchResult,LocNum,aveFeatureNum] = MatchFourImage(imageCurrent,imageNext,imorder,featureFigureSavePath,maxPlotFeatureN)
%% 匹配四幅图的特征点
% 输入：前后两帧的四幅图 imageCurrent,imageNext 中分别存储了当前和下一时刻的左右两个图片
% 由两种输入图像的方法，一种是直接输入 imread 得到的图片信息，一种是读取已经将两幅图匹配好的匹配结果
    % imageCurrent 为结构体，包含2个成员时为一种：imageCurrent.(leftImageCurrent,rightImageCurrent)
    % 包含4个成员时为一种：imageCurrent.(ldot,rdot,ldes,rdes)
% 必须同时在这四幅图中的匹配特征点才有效 
% 输出 ：同时在四幅图中匹配的sift特征点，分别在四幅图中的像素坐标
    % aveFeatureNum ：4副图的平均特征点个数

%% 调用两幅图的匹配函数 MatchTwoImage 先分别匹配前后时刻的左右图片
%if ~isfield(imageCurrent,'ldot')    % imageCurrent 中为 imread 的图片 
if imorder==1
    [ldot1,rdot1,ldes1,rdes1,aveFeatureNumCurrent,lAllLoc1,rAllLoc1] = MatchTwoImage(imageCurrent.leftImageCurrent,imageCurrent.rightImageCurrent,imorder,featureFigureSavePath,maxPlotFeatureN);
else    % imageCurrent 中为 匹配好的特征点
    ldot1 = imageCurrent.ldot ;
    rdot1 = imageCurrent.rdot ;
    ldes1 = imageCurrent.ldes ;
    rdes1 = imageCurrent.rdes ;
    lAllLoc1 = imageCurrent.lAllLoc;
    rAllLoc1 = imageCurrent.rAllLoc;
    aveFeatureNumCurrent = imageCurrent.aveFeatureNum ;
end 
[ldot2,rdot2,ldes2,rdes2,aveFeatureNumNext,lAllLoc2,rAllLoc2] = MatchTwoImage(imageNext.leftImageNext,imageNext.rightImageNext,imorder+1,featureFigureSavePath,maxPlotFeatureN);

% 输入后一时刻的匹配结果，供下一时刻直接调用
NextTwoMatchResult.ldot = ldot2 ;
NextTwoMatchResult.rdot = rdot2 ;
NextTwoMatchResult.ldes = ldes2 ;
NextTwoMatchResult.rdes = rdes2 ;
NextTwoMatchResult.aveFeatureNum = aveFeatureNumNext ;
NextTwoMatchResult.lAllLoc = lAllLoc2 ;
NextTwoMatchResult.rAllLoc = rAllLoc2 ;
aveFeatureNum = fix((aveFeatureNumCurrent + aveFeatureNumNext)/2);
%% 匹配前后两个时刻左右图片的结果
distRatio = 0.6;   

ldes2t = ldes2';                          % Precompute matrix transpose
trackl = zeros(1,size(ldes1,1));          % Declare array space to sign match points
matched_num = 0;                                  % Store the number of match points

locl1 = zeros(400,2);
locl2 = zeros(400,2);
locr1 = zeros(400,2);
locr2 = zeros(400,2);

% 加入唯一性约束
flag1 = zeros(1,size(ldes2,1));
flag2 = zeros(1,size(rdes2,1));
flag3 = zeros(1,size(rdes2,1));

for i = 1 : size(ldes1,1)
   dotprods = ldes1(i,:) * ldes2t;        % Computes vector of dot products
   [vals,indx] = sort(acos(dotprods));  % Take inverse cosine and sort results
   
   if (vals(1) < distRatio * vals(2)) && flag1(indx(1)) == 0
      trackl(i) = indx(1);
      flag1(indx(1)) = 1;

      rdes2t = rdes2';                          % Precompute matrix transpose
      trackr = zeros(1,size(rdes1,1));          % Declare array space to sign match points
      dotprods = rdes1(i,:) * rdes2t;        % Computes vector of dot products
      [vals,indx] = sort(acos(dotprods));  % Take inverse cosine and sort results
 
      if (vals(1) < distRatio * vals(2)) && flag2(indx(1)) == 0
          trackr(i) = indx(1);
          flag2(indx(1)) = 1;
 
          dotprods = ldes2(trackl(i),:) * rdes2t;
          [vals,indx] = sort(acos(dotprods));  % Take inverse cosine and sort results
          
          if indx(1) == trackr(i) && vals(1) < distRatio * vals(2) && flag3(indx(1)) == 0
              matched_num = matched_num + 1;
              locl1(matched_num,:) = ldot1(i,:);
              locr1(matched_num,:) = rdot1(i,:);
              locl2(matched_num,:) = ldot2(trackl(i),:);
              locr2(matched_num,:) = rdot2(trackr(i),:);
              flag3(indx(1)) = 1;
          end
      end
   end
end
locl1 = locl1(1:matched_num,:);
locr1 = locr1(1:matched_num,:);
locl2 = locl2(1:matched_num,:);
locr2 = locr2(1:matched_num,:);
%% 输出结果
LocNum = matched_num;
leftLocCurrent = locl1;
rightLocCurrent = locr1 ;
leftLocNext = locl2 ;
rightLocNext = locr2 ;
%% 绘图
% Program below showing images and tracked dots
global doFigureNum
firstStr = sprintf('first time two:%d(%0.2f%%)',length(ldot1),100*matched_num/length(ldot1));
secondStr = sprintf('second time two:%d(%0.2f%%)',length(ldot2),100*matched_num/length(ldot2));
matchedStr = sprintf('matched num:%d',matched_num);
str1 = sprintf('%s  %s  %s',matchedStr,firstStr,secondStr);

firstLeftStr = sprintf('first left:%d(%0.2f%%)',length(lAllLoc1),100*matched_num/length(lAllLoc1));
firstRightStr = sprintf('first Right:%d(%0.2f%%)',length(rAllLoc1),100*matched_num/length(rAllLoc1));
secondLeftStr = sprintf('second left:%d(%0.2f%%)',length(lAllLoc2),100*matched_num/length(lAllLoc2));
secondRightStr = sprintf('second Right:%d(%0.2f%%)',length(rAllLoc2),100*matched_num/length(rAllLoc2));
str2 = sprintf('%s  %s  %s',firstLeftStr,firstRightStr,secondLeftStr,secondRightStr);

disp([num2str(imorder),'时刻'])
disp(str1)
disp(str2)

if doFigureNum==-1 || imorder<doFigureNum || mod(imorder,10)==0
    % 左右匹配结果图
    imCL = imageCurrent.leftImageCurrent ;
    imCR = imageCurrent.rightImageCurrent ;
    imNL = imageNext.leftImageNext ;
    imNR = imageNext.rightImageNext ;

    imFour = [imCL imCR;imNL imNR];
    h_imFour = figure('name',['左右前后匹配图-',num2str(imorder)],'Position', [100 100 size(imFour,2) size(imFour,1)]);
    colormap('gray');   % 虽然压缩后像素大小会变，但是 figure 会将原图的像素个数转化为XY坐标大小，绘特征点图可直接用特征点像素坐标
    imagesc(imFour);
    imXLenght = size(imCL,2);
    imYLenght = size(imCL,1);
    hold on;
    if maxPlotFeatureN~=0
        NdesToDis = min(matched_num,maxPlotFeatureN) ; 
    else
        NdesToDis = matched_num;
    end
    % 标记成功匹配的点
    for i = 1: NdesToDis 
        plot(locl1(i,2),locl1(i,1),'.','color','red');              % 前左 上左
        plot(locl2(i,2),locl2(i,1)+imYLenght,'.','color','red');    % 后左 下左
        plot(locr1(i,2)+imXLenght,locr1(i,1),'.','color','red');    % 前右 上右
        plot(locr2(i,2)+imXLenght,locr2(i,1)+imYLenght,'.','color','red');  % 后右 下右
        line([locl1(i,2),locr1(i,2)+imXLenght],[locl1(i,1),locr1(i,1)],'Color','c');    % 前左右
        line([locl1(i,2),locl2(i,2)],[locl1(i,1),locl2(i,1)+imYLenght],'Color','c');    % 前后左
        line([locr1(i,2)+imXLenght,locr2(i,2)+imXLenght],[locr1(i,1),locr2(i,1)+imYLenght],'Color','c');    % 前后右
        line([locl2(i,2),locr2(i,2)+imXLenght],[locl2(i,1)+imYLenght,locr2(i,1)+imYLenght],'Color','c');    % 后左右
    end
    line([0 2*imXLenght],[imYLenght imYLenght],'Color','g','LineWidth',2);
    line([imXLenght imXLenght],[0 imYLenght*2],'Color','g','LineWidth',2);

    
    text(0,0,str1,'Color','m');

    
    text(0,50,str2,'Color','m');

    saveas(h_imFour,[featureFigureSavePath,'\前后左右图-',num2str(imorder),'.jpg'])
    saveas(h_imFour,[featureFigureSavePath,'\前后左右图-',num2str(imorder),'.fig'])
    close(gcf)
end



function [ldot, rdot, ldes, rdes,aveFeatureNum,loc1,loc2] = MatchTwoImage(imageLeft, imageRight,imorder,featureFigureSavePath,maxPlotFeatureN)
%% 输入：两个图像imageFile1（左）和imageFile2（右）

%% 输出
% ldot：左图中的匹配点[598个匹配特征点时为598*2]  rdot：右图中的匹配点[598个匹配特征点时为598*2]
% 特征点按匹配顺序存储：ldot(k,:) 和 rdot(k,:)是匹配的
% ldot(k,1)是Y方向（从上往下）像素坐标，ldot(k,2)是X方向（从左往右）像素坐标。
%   Plot绘制特征点时，根据默认的图片坐标，应该用 plot(ldot(k,2),ldot(k,1),'.','red')
% ldes：左图中的匹配点的sift参数[598个匹配特征点时为598*128]  rdot：右图中的匹配点的sift参数[598个匹配特征点时为598*128]
% aveFeatureNum 输入的两幅图的平均特征点个数
%%

disp([num2str(imorder),'时刻 sift中... '])
t1=toc ;
[imL, des1, loc1] = sift(imageLeft);
disp('imageRight - sift 开始')
[imR, des2, loc2] = sift(imageRight);
t2 = toc-t1 ;
disp([num2str(imorder),'时刻 siftOK: ',num2str(t2)])
% des1 为image1中提取出的sift特征点的sift参数[例-2391*128]
% loc1 为sift特征点的位置坐标[例-2391*4]，前两列为像素坐标，后两列是
aveFeatureNum = fix(( length(loc1)+length(loc2) )/2) ;
distRatio = 0.6;   % 这个参数用于控制匹配的准确度吧？具体意义是多少？取多少合适

des1t = des1';
des2t = des2';                          % Precompute matrix transpose
match = zeros(1,size(des1,1));          % Declare array space to sign match points
matched_num=0;                                  % Store the number of match points

%%%%%%%%%% add uniqueness and corresponding constraint %%%%%%%%%%
flag2 = zeros(1,size(des2,1));

for i = 1 : size(des1,1)
   dotprods = des1(i,:) * des2t;        % Computes vector of dot products
   [vals,indx] = sort(acos(dotprods));  % Take inverse cosine and sort results
   
   if (vals(1) < distRatio * vals(2)) && flag2(indx(1)) == 0
      dotprods2 = des2(indx(1),:) * des1t;
      [vals2,indx2] = sort(acos(dotprods2));
      if (vals2(1) < distRatio * vals2(2)) && indx2(1) == i  % 唯一性约束
          match(i) = indx(1);
          matched_num = matched_num + 1;
          flag2(indx(1)) = 1;
      else
          match(i) = -1;
      end
   else
       match(i) = -2;
   end
end
% 以des1的维数为基础（左图的特征点个数），检查左图每个特征点对应的匹配点坐标
% match(i)为左图第i个点的对应右图的坐标序号，不存在匹配点则置0

%% 存匹配结果
ldot = zeros(matched_num,2);
ldes = zeros(matched_num,128);
inum = 0;
% 以 imageLeft 特征点顺序为基准进行匹配的
for i=1:size(des1,1)
    if(match(i)>0)
        inum = inum + 1;
        ldot(inum,:) = loc1(i,1:2);
        ldes(inum,:) = des1(i,:);
    end
end

%%
rdot = zeros(matched_num,2);
rdes = zeros(matched_num,128);
inum = 0;
% imageFileRight的特征点安装匹配顺序重新排列
for i=1:size(des1,1)
    if(match(i)>0)
        inum = inum + 1;
        rdot(inum,:) = loc2(match(i),1:2);
        rdes(inum,:) = des2(match(i),:);
    end
end

%% 绘制原始特征点图像
global doFigureNum

if doFigureNum==-1 || imorder<doFigureNum  || mod(imorder,30)==0
    % 左右匹配结果图
    imLandR = appendimages(imL,imR);
    h_imLandR = figure('name',['左右匹配图-',num2str(imorder)],'Position', [100 100 size(imLandR,2) size(imLandR,1)]);
    colormap('gray');   % 虽然压缩后像素大小会变，但是 figure 会将原图的像素个数转化为XY坐标大小，绘特征点图可直接用特征点像素坐标
    imagesc(imLandR);
    im1XLenght = size(imL,2);
    hold on;
    if maxPlotFeatureN~=0
        NdesToDis = min(matched_num,maxPlotFeatureN) ; 
    else
        NdesToDis = matched_num;
    end
    % 标记成功匹配的点
    for i = 1: NdesToDis 
        plot(ldot(i,2),ldot(i,1),'.','color','red');
        plot(rdot(i,2)+im1XLenght,rdot(i,1),'.','color','red');
        line([ldot(i,2),rdot(i,2)+im1XLenght],[ldot(i,1),rdot(i,1)],'Color','c');
    end

    leftStr = sprintf('left:%d/%d(%0.2f%%)',matched_num,length(loc1),100*matched_num/length(loc1));
    rightStr = sprintf('right:%d/%d(%0.2f%%)',matched_num,length(loc2),100*matched_num/length(loc2));

    text(0,0,leftStr,'Color','m');
    text(0+im1XLenght,0,rightStr,'Color','m');
    hold off
    saveas(h_imLandR,[featureFigureSavePath,'\左右图-',num2str(imorder),'.fig'])
    saveas(h_imLandR,[featureFigureSavePath,'\左右图-',num2str(imorder),'.jpg'])
    close(gcf)
    %% 左图所有特征点
    h_imL = figure('name',['左图-',num2str(imorder)],'Position', [100 100 size(imL,2) size(imL,1)]);
    colormap('gray');
    imagesc(imL);
    hold on; 
    % 标记成功匹配的点
    for i = 1: length(loc1) 
        if match(i) == 0
            plot(loc1(i,2),loc1(i,1),'.','color','green');
        else
            plot(loc1(i,2),loc1(i,1),'.','color','red');
        end
    end
    text(0,0,leftStr,'Color','m');
    hold off
    saveas(h_imL,[featureFigureSavePath,'\左图-',num2str(imorder),'.fig'])
    saveas(h_imL,[featureFigureSavePath,'\左图-',num2str(imorder),'.jpg'])
    close(gcf)
    %% 右图所有特征点
    h_imR = figure('name',['右图-',num2str(imorder)],'Position', [100 100 size(imR,2) size(imR,1)]);
    colormap('gray');
    imagesc(imR);
    hold on; 
    % 标记成功匹配的点
    for i = 1: length(loc2) 
        plot(loc2(i,2),loc2(i,1),'.','color','green');
    end
    for i = 1: length(rdot) 
        plot(rdot(i,2),rdot(i,1),'.','color','red');
    end
    text(0,0,rightStr,'Color','m');
    saveas(h_imR,[featureFigureSavePath,'\右图-',num2str(imorder),'.fig'])
%     saveas(h_imR,[featureFigureSavePath,'\右图-',num2str(imorder),'.jpg'])
    close(gcf)

end
disp(['MatchTwoImage OK： ',num2str(imorder)])