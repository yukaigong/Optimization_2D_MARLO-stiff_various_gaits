function obj = changeGuardDir(obj, newDir)
    % changeGuardDir - change the direction of the guard evaluation
    %
    % Copyright 2014 Texas A&M University AMBER Lab
    % Author: Ayonga Hereid <ayonga@tamu.edu>
    
    % argument check
    narginchk(2,2);
    
    fprintf('Old guard direction: %d\n', obj.guardDir);
    obj.guardDir = newDir;
    fprintf('New guard direction: %d\n', newDir);
    
           


end