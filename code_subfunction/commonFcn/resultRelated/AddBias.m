% buaaxyz 2014.1.8

% ���ַ������ַ���_ ǰ�� \

function strOut = AddBias(strIn)
if iscell(strIn)
    for n=1:numel(strIn);
        k=1;
        while k<length(strIn{n})
            if strcmp(strIn{n}(k),'_') 
                strIn{n} = [strIn{n}(1:k-1),'\',strIn{n}(k:length(strIn{n}))];
                k=k+1;
            end
            k=k+1;
        end
        
    end
else
    
    k=1;
    while k<length(strIn)
        if strcmp(strIn(k),'_') 
            strIn = [strIn(1:k-1),'\',strIn(k:length(strIn))];
            k=k+1;
        end
        k=k+1;
    end
end
strOut = strIn;
