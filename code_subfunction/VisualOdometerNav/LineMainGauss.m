%%%%%利用列主元素Gauss消去法解方程组%%%%%
function X = LineMainGauss(A,b)
% 增广矩阵
n = size(b,1);
A = [A,b];
% 消元过程
for k = 1:n-1
    % 选主元
    [~,I] = max(abs(A(k:n,k)));
    Ik = I + k -1; % 主元行号
    for j = k:n+1
        tmp = A(k,j);
        A(k,j) = A(Ik,j);
        A(Ik,j) = tmp;
    end
    for i = k+1:n
        m = A(i,k) / A(k,k);
        for j = k+1:n
            A(i,j) = A(i,j) - m * A(k,j);
        end
        A(i,n+1) = A(i,n+1) - m * A(k,n+1);
    end
end
% 回代过程
X = zeros(n,1);
X(n) = A(n,n+1) / A(n,n);
for k = n-1:-1:1
    sum = 0;
    for j = k+1:n
        sum = sum + A(k,j) * X(j);
    end
    X(k) = (A(k,n+1) - sum) / A(k,k);
end
