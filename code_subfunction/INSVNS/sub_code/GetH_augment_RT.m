% buaa xyz 2014.1.17
% 源至白

% 计算量测矩阵：增广状态方程，q,T为量测
% 输入：
%       状态变量X  X=[q,r,v,gryoDrift,accDrift,q_last,r_last]
% 输出：雅克比量测矩阵

function H = GetH_augment_RT(X)

a0 = X(17);
a1 = X(18);
a2 = X(19);
a3 = X(20);
b0 = X(1);
b1 = X(2);
b2 = X(3);
b3 = X(4);

H1 = [ a0  a1  a2  a3;
      -a1  a0  a3 -a2;
      -a2 -a3  a0  a1;
      -a3  a2 -a1  a0];

H2(1,1) = b0;
H2(1,2) = b1;
H2(1,3) = b2;
H2(1,4) = b3;

H2(2,1) = b1;
H2(2,2) = - b0;
H2(2,3) = - b3;
H2(2,4) = b2;

H2(3,1) = b2;
H2(3,2) = b3;
H2(3,3) = - b0;
H2(3,4) = - b1;

H2(4,1) = b3;
H2(4,2) = - b2;
H2(4,3) = b1;
H2(4,4) = - b0;

Crb = [b0^2+b1^2-b2^2-b3^2,2*(b1*b2+b0*b3),2*(b1*b3-b0*b2);
       2*(b1*b2-b0*b3),b0*b0-b1*b1+b2*b2-b3*b3,2*(b2*b3+b0*b1);
       2*(b1*b3+b0*b2),2*(b2*b3-b0*b1),b0*b0-b1*b1-b2*b2+b3*b3];
   
H3 = - Crb;
H5 = Crb;

dpx = X(21) - X(5);
dpy = X(22) - X(6);
dpz = X(23) - X(7);

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

H = [H1,zeros(4,12),    H2,     zeros(4,3);
     H4,    H3,     zeros(3,13),     H5   ];
