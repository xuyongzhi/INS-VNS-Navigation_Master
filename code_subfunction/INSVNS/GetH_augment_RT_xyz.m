% buaa xyz 2014.1.17
% 源至白

% 计算量测矩阵：增广状态方程，直接法，q,T为量测
% 输入：
%       Qrb:当前时刻的姿态四元数
%       Qrb_last：上一组合时刻的姿态四元数
%       dPosition：上一组合时刻位置减去当前时刻的位置  r(k)-r(k+1)
% 输出：雅克比量测矩阵

function H = GetH_augment_RT_xyz(Qrb,Qrb_last,dPosition)

a0 = Qrb_last(1);
a1 = Qrb_last(2);
a2 = Qrb_last(3);
a3 = Qrb_last(4);
b0 = Qrb(1);
b1 = Qrb(2);
b2 = Qrb(3);
b3 = Qrb(4);

%  r(k)-r(k+1)
dpx = dPosition(1) ;
dpy = dPosition(2) ;
dpz = dPosition(3) ;

H1 = [ a0  a1  a2  a3;
      -a1  a0  a3 -a2;
      -a2 -a3  a0  a1;
      -a3  a2 -a1  a0];         % xyz验
H2 = [  b0  b1  b2  b3; 
        b1 -b0 -b3  b2;
        b2  b3 -b0 -b1;
        b3 -b2  b1 -b0  ] ;     % xyz验

Crb = FQtoCnb(Qrb);
H3 = - Crb;
H5 = Crb;

H4(1,1) = 2*b0*dpx + 2*b3*dpy - 2*b2*dpz;
H4(1,2) = 2*b1*dpx + 2*b2*dpy + 2*b3*dpz;
H4(1,3) = -2*b2*dpx + 2*b1*dpy - 2*b0*dpz;
H4(1,4) = -2*b3*dpx + 2*b0*dpy + 2*b1*dpz;

H4(2,1) = -2*b3*dpx + 2*b0*dpy + 2*b1*dpz;
H4(2,2) = 2*b2*dpx - 2*b1*dpy + 2*b0*dpz;
H4(2,3) = 2*b1*dpx + 2*b2*dpy + 2*b3*dpz;
H4(2,4) = -2*b0*dpx - 2*b3*dpy + 2*b2*dpz;

H4(3,1) = 2*b2*dpx - 2*b1*dpy + 2*b0*dpz;
H4(3,2) = 2*b3*dpx - 2*b0*dpy - 2*b1*dpz;
H4(3,3) = 2*b0*dpx + 2*b3*dpy - 2*b2*dpz;
H4(3,4) = 2*b1*dpx + 2*b2*dpy + 2*b3*dpz;

H = [H1,zeros(4,12),    zeros(4,4),     zeros(4,3);
     H4,    H3,     zeros(3,13),     zeros(3,3)   ];
