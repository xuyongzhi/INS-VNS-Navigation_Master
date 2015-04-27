% buaa xyz 2014.1.13

% 将 commonFcn 中的各个文件夹加入搜索路径中


addpath([pwd,'\resultRelated'])
addpath([pwd,'\resultRelated\ResultDisplay'])
addpath([pwd,'\elmat'])
addpath([pwd,'\dataHandle'])
addpath([pwd,'\coordinate'])
addpath([pwd,'\vision'])
addpath([pwd,'\kitti'])
addpath([pwd,'\kitti\devkit'])
addpath([pwd,'\kitti\libviso2\matlab'])
addpath([pwd,'\toolbox_calib'])
disp('commonFcn 加载到搜索路径中OK')