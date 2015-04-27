% buaa xyz 2014.1.8

% 使数组行存储/列存储

function new = makeRow(old,format,n)

%%　输入
% format：格式
% n：（可选输入）数据分量个数（当数据分量个数大于时刻数是必须指定）

if ~strcmp(format,'列为时') && ~strcmp(format,'行为时')
   errordlg('请指定格式') 
   new=[];
   return
end
if strcmp(format,'列为时')
    % 使为 一列一个时刻
    if exist('n','var')
       momentNum = n;       % 分量个数
    else
        momentNum = min(size(old));
    end
    if size(old,2)==momentNum
        new = old ;
    else        
       new = old'; 
    end
else
    % 一行一个时刻
    if exist('n','var')
       momentNum = n;       % 分量个数
    else
        momentNum = min(size(old));
    end
    if size(old,1)==momentNum
        new = old ;
    else
        new = old'; 
    end
end




    