function obj = setHolonomicConstraints(obj, constraints)
    % setDomainConstraints - configure holonomic constraints of domain
    %
    % Copyright 2014 Texas A&M University AMBER Lab
    % Author: Ayonga Hereid <ayonga@tamu.edu>
    
    % argument check
    narginchk(2,2);
    
    obj.holConstrName  = constraints;
    obj.nHolConstr   = numel(constraints);
    
    TYPE = 2;
    
    % check existance of mex files
%     assert(exist(strcat('hol_',obj.domainName),'file')==TYPE,...
%         'The MEX file could not be found: %s\n',...
%         strcat('hol_',obj.domainName)); % 3 - Mex-file
%     assert(exist(strcat('Jhol_',obj.domainName),'file')==TYPE,...
%         'The MEX file could not be found: %s\n',...
%         strcat('Jhol_',obj.domainName)); % 3 - Mex-file
%     assert(exist(strcat('dJhol_',obj.domainName),'file')==TYPE,...
%         'The MEX file could not be found: %s\n',...
%         strcat('dJhol_',obj.domainName)); % 3 - Mex-file
%     
%     obj.holConstrFunc = str2func(strcat('hol_',obj.domainName));
%     obj.holConstrJac = str2func(strcat('Jhol_',obj.domainName));
%     obj.holConstrJacDot = str2func(strcat('dJhol_',obj.domainName));
    
           

end
