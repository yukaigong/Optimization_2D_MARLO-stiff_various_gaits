function [ b ] = VirtualConstraints_Bezier( M, n )
%BEZIERPOLYNOMIALS 

syms s
a = sym('alpha', [n,M+1]);
b = sym(zeros(n,1));

for i=1:n
    for k=0:M 
         b(i) = b(i) + a(i,k+1) * (factorial(M)/(factorial(k)*factorial(M-k))) * s^k * (1-s)^(M-k);
    end
end

