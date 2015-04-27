%% 分析单步误差的特性
% buaa xyz 2014 5 22
% 输入一个误差变量（3*n）
% 输出 均值，方差，最大值

function [errorStr,errorMean,errorStd,errrMax] = AnalysSingleStepErorr(errorData)
N = length(errorData);
errorData = errorData*1e3; % m->mm
if N==size(errorData,1)
   errorData = errorData'; 
end
errorMean = mean(errorData,2);
errorStd = std(errorData,0,2);
errrMax = max(abs(errorData),[],2);
errorSum = sum(errorData,2);

errorMeanStr = sprintf('%0.2g ',errorMean);
errorStdStr = sprintf('%0.2g ',errorStd);
errrMaxStr = sprintf('%0.2g ',errrMax);
errorSumStr = sprintf('%0.2g ',errorSum);
errorStr = sprintf('\t(mm)平均值：%s\t方差:%s\t最大值：%s\t累积：%s',errorMeanStr,errorStdStr,errrMaxStr,errorSumStr);

