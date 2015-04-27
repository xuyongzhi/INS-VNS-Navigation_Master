% buaa xyz 2013.12.31

% 静态数据提出异常

function new_data = RejectUnusual_static( old_data,absErrorNeed )
format long
%% 功能： 对矢量数据(二维数组)去除异常数据
% absErrorNeed：绝对误差  （绝对和相对只需满足一个即可）
% 判断行存储还是列存储：长的为时间列
ErrorNum = 0 ;  %别去除的时刻数
data_size = size(old_data) ;
%如果为行存储，转为列存储（一列一个时刻）
if data_size(1)>data_size(2)
    old_data = old_data' ;
    data_size = size(old_data) ;
end
new_data = zeros(data_size) ;
for k = 1:data_size(1)    %列存储（一列一个时刻）
    data_k = old_data(k,:) ;
    new_data_k = RejectUnusual_Scalar( data_k,absErrorNeed(k) ) ;
    new_data(k,:) = new_data_k ;
end
% 去掉异常数据：保持时间同步性，每列只要有一个异常则整列去除
i = 1;
while i<=length(new_data)
   %一列列检查
   temp = isnan( new_data(:,i) ) ;
   if sum( temp ) ~= 0      %第i列含有 nan
       new_data(:,i) = [] ;     %去除第i列
       ErrorNum =ErrorNum +1 ;
   else
       i = i+1 ;
   end
end
disp('被去除的时刻数')
display(ErrorNum)

function  new_data  = RejectUnusual_Scalar( old_data,absErrorNeed )
%% 功能： 对标量数据 标注 异常数据：一维数组（异常则标注为 nan）
% 输出：
    % new_data：新的数据
    % ErrorNum：所有异常数据的个数
ErrorNum = 0 ;
data_size = size(old_data) ;
if min(data_size)>1
   errordlg('输入的数据不是一维(RejectUnusual_Scalar)') 
   return 
end
N_data = length(old_data) ;
new1_data = zeros(1,N_data) ;
data_temp = old_data ;

for k=1:5
     MaxError = absErrorNeed*(6-k)*5 ;    %允许最大误差
     data_ave = mean(old_data) ;
     for i=1:length(old_data)
         error_i = abs(old_data(i)-data_ave) ;   %i步误差
        if error_i>MaxError
            new1_data(i) = nan ;
            data_temp(i) = 0 ;
            ErrorNum = ErrorNum+1 ;
        else
            new1_data(i) = old_data(i);
        end   
     end
    
end
new_data = new1_data;
