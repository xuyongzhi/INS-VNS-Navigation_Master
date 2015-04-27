%% q¡£p=M(q)p
function M = GetQM(Q)

M = [ Q(1) -Q(2:4)';
     Q(2:4) Q(1)*eye(3)+getCrossMarix(Q(2:4)) ];

