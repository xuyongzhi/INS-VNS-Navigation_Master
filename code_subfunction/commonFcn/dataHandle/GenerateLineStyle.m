% buaa xyz 2013.12.25
% 生成线性参数

function [lineStyles,colors,makers] = GenerateLineStyle(n)
basicLineStyles = cellstr( char('-','--','-.',':') );
%basicMakers = cellstr( char('','*','o','+','x','.','s','d','^','v','>') );
basicMakers = cellstr( char('','','','','','','o','+','x','s','d','^','v','>') );

lineStyles = repmat(basicLineStyles,ceil(n/length(basicLineStyles)),1);
makers = repmat(basicMakers,ceil(n/length(basicMakers)),1);

basic_colors = cellstr( char('r','b','g','m','k','w','y','c') );
colors = repmat(basic_colors,ceil(n/length(basicMakers)),1);