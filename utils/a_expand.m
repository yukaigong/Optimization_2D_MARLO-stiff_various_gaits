function [ae] = a_expand(a)

if size(a, 2) == 5
    
    ae = zeros(size(a, 1), 7);
    ae(:, [1:4, 7]) = a;
    
else
    ae = a;
end

end
