% buaa xyz 2013.12.27

% 获取不同平率数据进行处理时。各个数据的有效长度

% 输入：数据的长度、频率（数组形式）
% 输出：数据的有效长度 （数组形式）
% 规则：起始点一致，组合数据频率取频率最小的分量数据

function [validLenthArray,combineK,combineLength,combineFre] = GetValidLength(lengthArrayOld,frequencyArray)

N = length(lengthArrayOld);  % 数据个数
validTime = (lengthArrayOld(1)-1)/frequencyArray(1);
for k=1:N
    validTime = min(validTime,(lengthArrayOld(k)-1)/frequencyArray(k)) ;
end
validLenthArray = zeros(1,N);
for k=1:N
    validLenthArray(k) = min( lengthArrayOld(k),fix(validTime*frequencyArray(k))+1 );
end
% 组合数据长度
% 组合数据取频率最小的那个数据
combineFre = min(frequencyArray) ;
combineK = 0;
for k=1:length(frequencyArray)
   if  combineFre==frequencyArray(k)
       combineK = k ;
       break;
   end
end
combineLength = validLenthArray(combineK);