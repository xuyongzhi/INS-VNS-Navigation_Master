function att = FRtoEularAngle(R)
format long
Crb = R;
% ÓÉ·½ÏòÓàÏÒ¾ØÕóÇó×ËÌ¬½Ç
att(1,1) = asin(Crb(2,3));  % ¸©Ñö½Ç
if Crb(3,3)>0
    att(2,1)=atan(-Crb(1,3)/Crb(3,3)); % roll
elseif Crb(3,3)<0
    if Crb(1,3)>0
        att(2,1)=att(2,1)-pi;
    else
        att(2,1)=att(2,1)+pi;
    end
elseif Crb(3,3)==0
    if Crb(1,3)>0
        att(2,1)=-pi/2;
    else
        att(2,1)=1/2*pi;
    end
end
if Crb(2,2)>0   % º½Ïò½Ç
    if Crb(2,1)>=0
        att(3,1) = atan(-Crb(2,1)/Crb(2,2)); % + 2 * pi
    elseif Crb(2,1)<0
        att(3,1) = atan(-Crb(2,1)/Crb(2,2));
    end
elseif Crb(2,2)<0
    att(3,1) = pi + atan(-Crb(2,1)/Crb(2,2));
elseif Crb(2,2)==0
    if Crb(2,1)>0
        att(3,1) = 1.5 * pi;
    elseif Crb(2,1)<0
        att(3,1) = pi / 2;
    end
end
