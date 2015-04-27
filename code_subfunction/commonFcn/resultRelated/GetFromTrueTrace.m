% buaa xyz 2014.1.9
% 2014.5.18 加  isempty(trueTrace)

% 从 trueTrace 中读取真实数据的具体信息

function  [planet,isKnowTrue,initialPosition_e,initialVelocity_r,initialAttitude_r,...
    trueTraeFre,true_position,true_attitude,true_velocity,true_acc_r,runTime_IMU,runTime_image] = GetFromTrueTrace( trueTrace )
    if ~isempty(trueTrace)
        true_position = trueTrace.position;
        true_attitude = trueTrace.attitude;
        if isfield(trueTrace,'velocity')
            true_velocity = trueTrace.velocity;
        else
            true_velocity=nan(3,length(true_position));
        end
        if isfield(trueTrace,'acc_r')
            true_acc_r = trueTrace.acc_r;
        else
            true_acc_r=[];
        end
        trueTraeFre = trueTrace.frequency;
        initialPosition_e = trueTrace.initialPosition_e ;
        initialVelocity_r = trueTrace.initialVelocity_r ;
        initialAttitude_r = trueTrace.initialAttitude_r ;
        planet = trueTrace.planet ;
        
        if isempty(true_position)
            isKnowTrue = 0 ;
        else
            isKnowTrue = 1 ;
        end
        
        if isfield(trueTrace,'runTime_IMU')
            runTime_IMU = trueTrace.runTime_IMU;
        else
            runTime_IMU=[];
        end
        if isfield(trueTrace,'runTime_image')
            runTime_image = trueTrace.runTime_image;
        else
            runTime_image=[];
        end
        
    else
        isKnowTrue = 0 ;
        true_position=[];
        true_attitude=[];
        true_velocity=[];
        true_acc_r=[];
        trueTraeFre=100;
        initialPosition_e=[];
        initialVelocity_r=[];
        initialAttitude_r=[];
        runTime_IMU=[];
        runTime_image=[];
        planet='m';
    end