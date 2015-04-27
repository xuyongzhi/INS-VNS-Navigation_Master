% buaa xyz 2014.1.9

% 从 trueTrace 中读取真实数据的具体信息

function  [planet,isKnowTrue,initialPosition_e,initialVelocity_r,initialAttitude_r,...
    trueTraeFre,true_position,true_attitude,true_velocity,true_acc_r] = GetFromTrueTrace( trueTrace )
    true_position = trueTrace.position;
    true_attitude = trueTrace.attitude;
    true_velocity = trueTrace.velocity;
    true_acc_r = trueTrace.acc_r;
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