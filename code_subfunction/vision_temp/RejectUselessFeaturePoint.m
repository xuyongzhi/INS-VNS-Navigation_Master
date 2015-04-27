%% 剔除匹配成功的特征点中左右图过于接近的特征点
% buaa xyz 2014 5 22
% aimFeatureN: 最小特征点个数，如果特征点个数小于这个则不进行视差检查
% maxdXin： 用于检查的最小视差
%%% 在特征点个数大于 aimFeatureN 前提下，视差小于maxdXin的特征点被剔除
function visualInputData = RejectUselessFeaturePoint(visualInputData,maxdXin,aimFeatureN)
disp('视差过小特征点检查中')
if ~exist('visualInputData','var')
   load('visualInputData.mat') 
end
if ~exist('maxdXin','var')
   maxdXin=4;    % 最小视差
end
if ~exist('aimFeatureN','var')
   aimFeatureN=20;    
end


leftLocCurrent = visualInputData.leftLocCurrent ;
rightLocCurrent = visualInputData.rightLocCurrent ;
leftLocNext = visualInputData.leftLocNext ;
rightLocNext = visualInputData.rightLocNext ;
matchedNum = visualInputData.matchedNum ;

invalidNum=0;

timeNum = length(leftLocCurrent);   % 图像采样时刻数
for i=1:timeNum
    if matchedNum(i)>300
        maxdX = maxdXin+2;
    elseif matchedNum(i)>150
        maxdX = maxdXin+1;
    else
        maxdX = maxdXin;
    end
   j=0;
   while j+1<=length(leftLocCurrent{i})
        j=j+1;
        xLc = [leftLocCurrent{i}(j,2);leftLocCurrent{i}(j,1)]; % 第i个时刻的第j个当前帧特征点，注意转置并交换顺序，因为原始数据为[y,x]
        xRc = [rightLocCurrent{i}(j,2);rightLocCurrent{i}(j,1)]; 
        dXc = abs(xLc-xRc) ;
        xLn = [leftLocNext{i}(j,2);leftLocNext{i}(j,1)]; % 第i个时刻的第j个下一帧帧特征点，注意转置并交换顺序，因为原始数据为[y,x]
    	xRn = [rightLocNext{i}(j,2);rightLocNext{i}(j,1)];
        dXn = abs(xLn-xRn) ;
        if dXc(1)<maxdX && dXc(2)<maxdX && dXn(1)<maxdX && dXn(2)<maxdX && length(leftLocCurrent{i}) > aimFeatureN
            invalidNum = invalidNum+1 ;
            matchedNum(i)=matchedNum(i)-1;
            leftLocCurrent{i}(j,:)=[];
            rightLocCurrent{i}(j,:)=[];
            leftLocNext{i}(j,:)=[];
            rightLocNext{i}(j,:)=[];
            j=j-1;
            str=sprintf('找到一个过于接近的特征点并丢弃，%d时刻的第%d个特征点',i,j);
%             disp(str)
        end
   end
   if mod(i,20)==0
    %  i 
   end
end

fprintf('被剔除特征点总个数：%d\n平均被剔除特征点个数：%0.1f\n',invalidNum,invalidNum/timeNum)
fprintf('处理后，特征点最少个数：%d',min(matchedNum))
visualInputData.leftLocCurrent = leftLocCurrent ;
visualInputData.rightLocCurrent = rightLocCurrent;
visualInputData.leftLocNext = leftLocNext;
visualInputData.rightLocNext = rightLocNext ;
visualInputData.matchedNum = matchedNum;

disp('视差过小特征点检查完成')