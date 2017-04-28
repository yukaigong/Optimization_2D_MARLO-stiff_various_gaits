function obj = setNextDomain(obj, next_domain)
    % setNextDomain - set next domain of the domain
    %
    % Copyright 2014 Texas A&M University AMBER Lab
    % Author: Ayonga Hereid <ayonga@tamu.edu>
    
    % argument check
    narginchk(2,2);
    
    % if the transition from the current domain to next domain involves
    % rigid impacts, then set the impact constraints to be holonomic
    % constraints of the next domain
    if obj.hasImpact && (next_domain.domainIndex ~= -1)
        obj.impConstrName = next_domain.holConstrName;
        obj.nImpConstr    = next_domain.nHolConstr;
%         obj.impConstrJac  = next_domain.holConstrJac;
    end
    
    
    nextDomain = struct();
    nextDomain.domainName  = next_domain.domainName;
    nextDomain.domainIndex = next_domain.domainIndex;
    
    obj.nextDomain = nextDomain;

end