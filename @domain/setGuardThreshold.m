function obj = setGuardThreshold(obj, value)
    % setGuardThreshold - set a new threshold for the guard condition
    %
    % Copyright 2014 Texas A&M University AMBER Lab
    % Author: Ayonga Hereid <ayonga@tamu.edu>
    
    % argument check
    narginchk(2,2);
    
    
    fprintf('updating the guard threshold: %f',value);
    obj.guardThld = value;
           


end