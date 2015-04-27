% buaaxyz 2014.1.3

% 不同频率数据的合并 时使用
% 功能：输入待合并的子数据的数据和频率，及组合数据的频率
%   输出子数据中提取的可用于直接合并的数据

function toCombineData  = GetDataToComine(subData,subFre,combineFre)
% subData 为一维，toCombineData与subData同存储形式
subLength = length(subData);
combineLength = fix(subLength*combineFre/subFre);
toCombineData = zeros(size(subData));
toCombineData = toCombineData(1:combineLength);

toCombineData(1) = subData(1) ;
for k=2:combineLength
    toCombineData(k) = subData(fix((k-1)/combineFre*subFre)+1);
end
