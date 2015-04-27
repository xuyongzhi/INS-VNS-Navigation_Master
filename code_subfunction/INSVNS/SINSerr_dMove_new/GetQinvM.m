%% q¡£p=M(p)q

function invM = GetQinvM(Q)

invM = [ Q(1) -Q(2:4)';
     Q(2:4) Q(1)*eye(3)-getCrossMarix(Q(2:4)) ];

