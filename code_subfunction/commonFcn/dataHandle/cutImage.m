
% 
function cutImage()

answer = inputdlg('图片剪切到','分辨率',1,{'1392 1040'});
res = sscanf(answer{1},'%d');
newPath = [pwd,'\cutedImage_',num2str(res(1)),'x',num2str(res(2)),'\'];
if isdir(newPath)
	delete([newPath,'\*']);
else
	mkdir(newPath); 
end

allImageFile = ls([pwd,'\*.bmp']);  % 所有左右相机图片的文件名
imNum = fix(size(allImageFile,1));

for k=1:imNum
    name=allImageFile(k,:);
    image = imread(name);
    image = image(1:res(2),1:res(1));
    
    newName = [newPath,name];
    newName = cutSpace(newName);
    imwrite(image,newName)
end
if imNum>0
    disp('bmp cutImage ok')
    imNum
end

allImageFile = ls([pwd,'\*.jpg']);  % 所有左右相机图片的文件名
imNum = fix(size(allImageFile,1));

for k=1:imNum
    name=allImageFile(k,:);
    image = imread(name);
    image = image(1:res(2),1:res(1));
    
    newName = [newPath,name];
    newName = cutSpace(newName);
    imwrite(image,newName)
end
if imNum>0
    disp('jpg cutImage ok')
    imNum
end

function newName = cutSpace(name)
k=0;
newName = name ;
for i=1:length(name)
   if  name(i)~=' '
       k=k+1;
   end
end

newName = newName(1:k);
