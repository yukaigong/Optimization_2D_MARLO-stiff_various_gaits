function obj = setResetMap(obj, delta)
    % setResetMap - configure reset map condition of domain
    %
    % Copyright 2014 Texas A&M University AMBER Lab
    % Author: Ayonga Hereid <ayonga@tamu.edu>
    
    % argument check
    narginchk(2,2);
    
    obj.hasImpact  = delta.impact;
    obj.isSwapping = delta.relabel; 
    
    

end