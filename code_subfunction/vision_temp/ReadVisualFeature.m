%% 读取视觉信息结构体visualInputData
%                               2014.8.7
%%% 输入：visualInputData 格式见说明文档
%%% 输出：四图成功匹配的特征点坐标
% 设：t个时刻（t对图片），t时刻的特征点个数是 N(t)
% 1> leftLocCurrent：cell{1*t}[N(t)*2] 前一时刻 左图 匹配成功特征点的 像素坐标
%               leftLocCurrent{k}(i,:) 为左图 第k时刻 第i个特征点 像素坐标
% 2> rightLocCurrent： 前一时刻 右图
% 3> leftLocNext： 后一时刻 左图
% 4> rightLocNext： 后一时刻 右图
% 5> matchedNum：[1*N] 特征点个数
%%% 像素坐标系的原点在左上角

function [ leftLocCurrent,rightLocCurrent,leftLocNext,rightLocNext,featureCPosCurrent,featureCPosNext,matchedNum ] = ReadVisualFeature(visualInputData)

%% 视觉数据
button=questdlg('是否进行最小视差检查?'); 
if strcmp(button,'Yes')
    minDx = 8 ;     % 特征点视差检查（1024x1024,45°角，景深=247/dX，dX=5时景深为49m，dX=6时41.2m，dX=10时24.7m，dX=12时21.6mdX=7时35m，dX=8时31m，dX=15时16.5m）
    disp(sprintf('最小视差检查：%d',minDx)) ; %#ok<DSPS>
    visualInputData = RejectUselessFeaturePoint(visualInputData,minDx);    
end
% 
leftLocCurrent = visualInputData.leftLocCurrent ;
rightLocCurrent = visualInputData.rightLocCurrent ;
leftLocNext = visualInputData.leftLocNext ;
rightLocNext = visualInputData.rightLocNext ;
featureCPosCurrent = visualInputData.featureCPosCurrent ;
featureCPosNext = visualInputData.featureCPosNext ;
matchedNum = visualInputData.matchedNum ;