%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 开始日期：2013.12.4
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
%imFormat = '.bmp';
imFormat = '.jpg';
%imFormat = {'.bmp','.jpg'};
% 得到输入数据的路径 inputDataPath_left inputDataPath_right
global projectDataPath leftPathName
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
[leftFileName, leftPathName] = uigetfile(['*',imFormat],'选择左相机的任意一幅图,要求图片名编号在最后，且只有编号为数字',inputDataPath_left);
if leftFileName==0
   return ;
end
[rightFileName, rightPathName] = uigetfile(['*',imFormat],'选择右相机的任意一幅图,要求图片名编号在最后，且只有编号为数字',inputDataPath_right);
if rightFileName==0
   return ;
end
[leftPrefix,leftSufffix] = GetFileFix(leftFileName) ;
[rightPrefix,rightSuffix] = GetFileFix(rightFileName) ;
%% 计算图片个数
if strcmp(leftPathName,rightPathName)==1
    allImageFile = ls([leftPathName,['*',imFormat]]);  % 所有左右相机图片的文件名
    imorder = fix(size(allImageFile,1)/2);
else
    leftImageFile = ls([leftPathName,leftPrefix,['*',imFormat]]);  % 所有左相机图片的文件名
    rightImageFile = ls([rightPathName,rightPrefix,['*',imFormat]]);
    imorder = min(size(leftImageFile,1),size(rightImageFile,1));   % 时刻数
end
disp(['时刻数： ',num2str(imorder)])
%% 匹配结果图片存储路径
global matchResultPath
matchResultPath = [GetUpperPath(GetUpperPath(GetUpperPath(leftPathName))),'\匹配结果'];
if(isdir(matchResultPath))
   rmdir(matchResultPath) ;
end
mkdir(matchResultPath);

featureFigureSavePath = [leftPathName,'\特征点匹配图'];
if isdir(featureFigureSavePath)
    delete([featureFigureSavePath,'\*']);
else
    mkdir(featureFigureSavePath); 
end
%% 结果变量
leftLocCurrent = cell(1,imorder-1);  % 存储左相机当前图匹配成功的特征点，一个细胞一个时刻
rightLocCurrent = cell(1,imorder-1);
leftLocNext = cell(1,imorder-1);
rightLocNext = cell(1,imorder-1);
matchedNum = zeros(1,imorder-1);
aveFeatureNum = zeros(1,imorder-1);
    % 每个细胞中一个结构体，注意两个坐标合到一行中
predictTime = imorder*20;   % 预计时间    
waitbar_h = waitbar(0,{'开始特征点提取与匹配...';['预计共需',num2str(predictTime),' s']});
tic
for i=1:imorder-1
    disp(['第 ',num2str(i),' / ',num2str(imorder-1),' 个时刻：',sprintf('%d',toc),'sec']);
   %% 提取一个时刻匹配成功的特征点：针对当前帧与下一帧的四副图片 
   %% 仅在i==1时需要提前当前
    imageCurrent = [];
    if i==1
        if ~exist([leftPathName,leftPrefix,num2str(i),leftSufffix],'file')
            disp(['缺少图片：',leftPathName,leftPrefix,num2str(i),leftSufffix])
        else
            leftImageCurrent = imread([leftPathName,leftPrefix,num2str(i),leftSufffix]);   
            rightImageCurrent = imread([rightPathName,rightPrefix,num2str(i),rightSuffix]);
            % 视景仿真图片的分辨率修改问题
            if i==1 
                isCut = 0 ;
                if length(leftImageCurrent)>1392
                    isCutStr = questdlg('是否裁剪','图片裁剪','是','否','是') ;
                    if strcmp(isCutStr,'是')
                        isCut = 1 ;
                        resolutionStr=inputdlg({'裁剪目标分辨率'},'裁剪',1,{'1392 1040'});
                        resolution = sscanf(resolutionStr{1},'%f');
                    end
                end
            end
            if isCut==1
                leftImageCurrent = leftImageCurrent(1:resolution(2),1:resolution(1),:);
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

            if ~isdir([leftPathName,'处理后的图片'])
               mkdir([leftPathName,'处理后的图片']) ;
            end
            if ~isdir([rightPathName,'处理后的图片'])
               mkdir([rightPathName,'处理后的图片']) ;
            end
            imwrite(leftImageCurrent,[leftPathName,'处理后的图片','\leftImage',num2str(i),'.jpg'],'jpg');
            imwrite(rightImageCurrent,[rightPathName,'处理后的图片','\rightImage',num2str(i),'.jpg'],'jpg');

            imageCurrent.leftImageCurrent = leftImageCurrent ;
            imageCurrent.rightImageCurrent = rightImageCurrent ;
        end
    else
        imageCurrent = NextTwoMatchResult ; % 取上次存储的匹配结果
    end
    %% 提下一时刻的图片
    if ~exist([leftPathName,leftPrefix,num2str(i+1),leftSufffix],'file')
    	disp(['缺少图片：',leftPathName,leftPrefix,num2str(i+1),leftSufffix])
    else
        leftImageNext = imread([leftPathName,leftPrefix,num2str(i+1),leftSufffix]);
        rightImageNext = imread([rightPathName,rightPrefix,num2str(i+1),rightSuffix]);
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
        imwrite(leftImageNext,[leftPathName,'处理后的图片','\leftImage',num2str(i+1),'.jpg'],'jpg');
        imwrite(rightImageNext,[rightPathName,'处理后的图片','\rightImage',num2str(i+1),'.jpg'],'jpg');
        %% 提取匹配特征点

        imageNext.leftImageNext = leftImageNext;
        imageNext.rightImageNext = rightImageNext;
        
        [leftLocCurrent{i},rightLocCurrent{i},leftLocNext{i},rightLocNext{i},NextTwoMatchResult,matchedNum(i),aveFeatureNum(i)] = MatchFourImage(imageCurrent,imageNext,i,featureFigureSavePath);
    end
   % if mod(i,ceil((imorder-1)/10)==0)
        waitbar(i/(imorder-1),waitbar_h,{['完成第 ',num2str(i),'/',num2str(imorder-1),' 个时刻，共用时：',sprintf('%0.1f',toc),'sec',['预计共需',num2str(predictTime),' s']]});
   % end
end
close(waitbar_h);
% 获取数据的频率
frequency = inputdlg('输入图片的采集频率');
frequency = str2double(frequency);
% 存储匹配点
visualInputData.leftLocCurrent = leftLocCurrent;
visualInputData.rightLocCurrent = rightLocCurrent;
visualInputData.leftLocNext = leftLocNext;
visualInputData.rightLocNext = rightLocNext;
visualInputData.matchedNum = matchedNum;
visualInputData.aveFeatureNum = aveFeatureNum;
visualInputData.frequency = frequency;

save([pwd,'\siftMatchResult\visualInputData.mat'],'visualInputData')  
assignin('base','visualInputData',visualInputData)
disp('图片的特征点提取结束，已保存到 siftMatchResult 文件和base空间')

function [prefix,suffix] = GetFileFix(filename)
% 搜索所有非数字部分
prefixNum = 1 ; % 记录非数字字符的个数

if ~ischar(filename(1))
   errordlg('没有前缀'); 
   return ;
end

for i=2:length(filename)
   if ~isNumStr(filename(i))  && ~isNumStr(filename(i-1))
       prefixNum = prefixNum+1 ;    % 找到一个字符  
   else
    	break;
   end
end
prefix = filename(1:prefixNum); % 前缀

for i=prefixNum+1:length(filename)
   if ~isNumStr(filename(i)) 
       break;
   end
end
suffixNum = i;
if(suffixNum==length(filename))
   suffix = [];     % 后缀
else
    suffix = filename(suffixNum:length(filename));
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

function [leftLocCurrent,rightLocCurrent,leftLocNext,rightLocNext,NextTwoMatchResult,LocNum,aveFeatureNum] = MatchFourImage(imageCurrent,imageNext,imorder,featureFigureSavePath)
%% 匹配四幅图的特征点
% 输入：前后两帧的四幅图 imageCurrent,imageNext 中分别存储了当前和下一时刻的左右两个图片
% 由两种输入图像的方法，一种是直接输入 imread 得到的图片信息，一种是读取已经将两幅图匹配好的匹配结果
    % imageCurrent 为结构体，包含2个成员时为一种：imageCurrent.(leftImageCurrent,rightImageCurrent)
    % 包含4个成员时为一种：imageCurrent.(ldot,rdot,ldes,rdes)
% 必须同时在这四幅图中的匹配特征点才有效 
% 输出 ：同时在四幅图中匹配的sift特征点，分别在四幅图中的像素坐标
    % aveFeatureNum ：4副图的平均特征点个数

%% 调用两幅图的匹配函数 MatchTwoImage 先分别匹配前后时刻的左右图片
if ~isfield(imageCurrent,'ldot')    % imageCurrent 中为 imread 的图片 
    [ldot1,rdot1,ldes1,rdes1,aveFeatureNumCurrent] = MatchTwoImage(imageCurrent.leftImageCurrent,imageCurrent.rightImageCurrent,imorder);
else    % imageCurrent 中为 匹配好的特征点
    ldot1 = imageCurrent.ldot ;
    rdot1 = imageCurrent.rdot ;
    ldes1 = imageCurrent.ldes ;
    rdes1 = imageCurrent.rdes ;
    aveFeatureNumCurrent = imageCurrent.aveFeatureNum ;
end 
if ~isfield(imageNext,'ldot')    % imageNext 中为 imread 的图片
    [ldot2,rdot2,ldes2,rdes2,aveFeatureNumNext] = MatchTwoImage(imageNext.leftImageNext,imageNext.rightImageNext,imorder);
else    % imageNext 中为 匹配好的特征点
    ldot2 = imageNext.ldot ;
    rdot2 = imageNext.rdot ;
    ldes2 = imageNext.ldes ;
    rdes2 = imageNext.rdes ;
end
% 输入后一时刻的匹配结果，供下一时刻直接调用
NextTwoMatchResult.ldot = ldot2 ;
NextTwoMatchResult.rdot = rdot2 ;
NextTwoMatchResult.ldes = ldes2 ;
NextTwoMatchResult.rdes = rdes2 ;
NextTwoMatchResult.aveFeatureNum = aveFeatureNumNext ;
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

%return
isShowFigure= 1;
if isShowFigure==1
        % Create a new image showing the two images side by side.
    lineNum = 20 ;  % 特征点连线数
	if isfield(imageCurrent,'leftImageCurrent')
        im2l = appendimages(imageCurrent.leftImageCurrent,imageNext.leftImageNext);
        % Show a figure with lines joining the accepted matches.
        figure('name','左-前后图','Position', [100 100 size(im2l,2) size(im2l,1)]);
                colormap('gray');
        imagesc(im2l);
        hold on;
        cols1 = size(imageCurrent.leftImageCurrent,2);
        plot(leftLocCurrent(:,2),leftLocCurrent(:,1),'.','color','red');
        hold on;
        plot(leftLocNext(:,2)+cols1,leftLocNext(:,1),'.','color','red');
        hold on;
        for i = 1: min(matched_num,lineNum)
            line([leftLocCurrent(i,2) leftLocNext(i,2)+cols1], ...
                 [leftLocCurrent(i,1) leftLocNext(i,1)], 'Color', 'c');
        end        
        hold off;
        title(['四图匹配的特征点个数：',num2str(matched_num)]);
        saveas(gcf,[featureFigureSavePath,'\左-前后图',num2str(imorder),'.fig']);
        disp(['保存了图片：前后左图',num2str(imorder),'.fig'])
     %   close(gcf)
        %%
        % Create a new image showing the two images side by side.
        im2r = appendimages(imageCurrent.rightImageCurrent,imageNext.rightImageNext);
        % Show a figure with lines joining the accepted matches.
        figure('name','右-前后图','Position', [100 100 size(im2r,2) size(im2r,1)]);
        colormap('gray');
        imagesc(imageCurrent.rightImageCurrent);
        hold on;
        cols1 = size(imageCurrent.rightImageCurrent,2);
        plot(rightLocCurrent(:,2),rightLocCurrent(:,1),'.','color','red');
        hold on;
        plot(rightLocNext(:,2)+cols1,rightLocNext(:,1),'.','color','red');
        hold on;
        for i = 1: min(matched_num,lineNum)

            line([rightLocCurrent(i,2) rightLocNext(i,2)+cols1], ...
                 [rightLocCurrent(i,1) rightLocNext(i,1)], 'Color', 'c');

        end
        hold off;
        title(['四图匹配的特征点个数：',num2str(matched_num)]);
        saveas(gcf,[featureFigureSavePath,'\右-前后图',num2str(imorder),'.fig']);
        disp(['保存了图片：前右左图',num2str(imorder),'.fig'])
    %    close(gcf)
        %%
        figure('name','左右图');
        im1 = [imageCurrent.leftImageCurrent,imageCurrent.rightImageCurrent];
        imshow(im1);hold on;
        plot(leftLocCurrent(:,2),leftLocCurrent(:,1),'.','color','red');
        hold on;
        plot(rightLocCurrent(:,2)+cols1,rightLocCurrent(:,1),'.','color','red');
        hold on;
        cols1 = size(imageCurrent.leftImageCurrent,2);
        for i = 1:min(matched_num,lineNum)
            line([leftLocCurrent(i,2) rightLocCurrent(i,2)+cols1], ...
                 [leftLocCurrent(i,1) rightLocCurrent(i,1)], 'Color', 'c');
        end
        title(['四图匹配的特征点个数：',num2str(matched_num)]);
        saveas(gcf,[featureFigureSavePath,'\左右图',num2str(imorder),'.fig']);
        disp(['保存了图片：左右图',num2str(imorder),'.fig'])
     %   close(gcf)
	end
    %%    
    figure('name','左右图');    
    im2 = [imageNext.leftImageNext,imageNext.rightImageNext];
    imshow(im2);
    hold on;
    plot(leftLocNext(:,2),leftLocNext(:,1),'.','color','red');
    hold on;
    cols1 = size(imageNext.leftImageNext,2);
    plot(rightLocNext(:,2)+cols1,rightLocNext(:,1),'.','color','red')
    hold on;
    
    for i = 1:min(matched_num,lineNum)
        line([leftLocNext(i,2) rightLocNext(i,2)+cols1], ...
             [leftLocNext(i,1) rightLocNext(i,1)], 'Color', 'c');
    end
    title(['四图匹配的特征点个数：',num2str(matched_num)]);
    saveas(gcf,[featureFigureSavePath,'\左右图',num2str(imorder),'.fig']);
    disp(['保存了图片：左右图',num2str(imorder),'.fig'])
  	close(gcf)
end

function [ldot, rdot, ldes, rdes,aveFeatureNum] = MatchTwoImage(imageFileLeft, imageFileRight,imorder)
%% 输入：两个图像imageFile1（左）和imageFile2（右）
global matchResultPath
if isempty(matchResultPath)
   matchResultPath=pwd; 
end
%% 输出
% ldot：左图中的匹配点[598个匹配特征点时为598*2]  rdot：右图中的匹配点[598个匹配特征点时为598*2]
% ldot 和 rdot中的特征点按匹配顺序存储
% ldes：左图中的匹配点的sift参数[598个匹配特征点时为598*128]  rdot：右图中的匹配点的sift参数[598个匹配特征点时为598*128]
% aveFeatureNum 输入的两幅图的平均特征点个数
%%
image1 = imageFileLeft;
image2 = imageFileRight;

disp('sift 开始')
t1=toc ;
[im1, des1, loc1] = sift(image1);
disp('image1 - sift OK')
t2 = toc-t1 
disp('image2 - sift 开始')
[im2, des2, loc2] = sift(image2);
t3 = toc-t1 
disp('image2 - sift OK')
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
          match(i) = 0;
      end
   end
end
% 以des1的维数为基础（左图的特征点个数），检查左图每个特征点对应的匹配点坐标
% match(i)为左图第i个点的对应右图的坐标序号，不存在匹配点则置0

%% 存匹配结果
ldot = zeros(matched_num,2);
ldes = zeros(matched_num,128);
inum = 0;
% 以 imageFileLeft 特征点顺序为基准进行匹配的
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

% return
%% 绘图
%% 绘制原始特征点图像
im3 = appendimages(im1,im2);
figure('Position', [100 100 size(im3,2) size(im3,1)]);
colormap('gray');
imagesc(im3);
cols1 = size(im1,2);
hold on;

%    NdesToDis = size(des1,1) ; % 所有特征点都显示
NdesToDis = 1000 ;
for i = 1: NdesToDis
    if i<=length(loc1) && i<=length(loc2)
      if (match(i) > 0)     % 匹配成功的特征点：红色
          plot(loc1(i,2),loc1(i,1),'.','color','red')
          plot(loc2(i,2),loc2(i,1),'.','color','red')      
      else                  % 匹配失败的特征点：绿色
          plot(loc1(i,2),loc1(i,1),'.','color','green')
          plot(loc2(i,2),loc2(i,1),'.','color','green')
      end
    end
end
leftStr = sprintf('left:%d/%d(%0.2f%%)',matched_num,length(loc1),100*matched_num/length(loc1));
text(105,105,leftStr);
rightStr = sprintf('right:%d/%d(%0.2f%%)',matched_num,length(loc2),100*matched_num/length(loc2));
text(105+cols1,105,rightStr);

%% 绘制匹配图像
im4 = appendimages(im1,im2);
figure('Position', [100 100 size(im4,2) size(im4,1)]);
colormap('gray');
imagesc(im4);
cols1 = size(im1,2);
hold on;
% 只显示匹配成功的特征点，并连线
line_num = 0 ;
for i = 1: size(des1,1)
	if (match(i) > 0)     % 匹配成功的特征点：红色
        plot(loc1(i,2),loc1(i,1),'.','color','red')
        plot(loc2(match(i),2),loc2(match(i),1),'.','color','red') 
        if line_num<30
            % 左图 (loc1(i,1),loc1(i,2)) 右图(loc2(match(i),1),loc2(match(i),2)) 连接这两点
            line([loc1(i,2) loc2(match(i),2)+cols1], ...
                [loc1(i,1) loc2(match(i),1)], 'Color', 'c');
            line_num=line_num+1;
        end
  end
end
leftStr = sprintf('left:%d/%d(%0.2f%%)',100*matched_num,length(loc1),100*matched_num/length(loc1));
text(105,105,leftStr);
rightStr = sprintf('right:%d/%d(%0.2f%%)',100*matched_num,length(loc2),100*matched_num/length(loc2));
text(105+cols1,105,rightStr);

% return
if isShowFigure==1
    % Create a new image showing the two images side by side.
    im3 = appendimages(im1,im2);

    % Show a figure with lines joining the accepted matches.
    figure('Position', [100 100 size(im3,2) size(im3,1)]);
    colormap('gray');
    imagesc(im3);
    hold on;
    cols1 = size(im1,2);
    for i = 1: size(des1,1)
      if (match(i) > 0)
          % 左图 (loc1(i,1),loc1(i,2)) 右图(loc2(match(i),1),loc2(match(i),2)) 连接这两点
        line([loc1(i,2) loc2(match(i),2)+cols1], ...
             [loc1(i,1) loc2(match(i),1)], 'Color', 'c');
      end
    end
    hold off;
end

if isShowFigure==1
    figure,imshow(image1)
    hold on
    for i=1:size(des1,1)
        if(match(i)>0)
            inum = inum + 1;
            plot(loc1(i,2),loc1(i,1),'.','color','red')
        end
    end
end


if isShowFigure==1
    figure,imshow(image2)
    hold on
    for i=1:size(des1,1)
        if(match(i)>0)
            plot(loc2(match(i),2),loc2(match(i),1),'.','color','red')
        end
    end
end


