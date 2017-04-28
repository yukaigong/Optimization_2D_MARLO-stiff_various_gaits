function obj = setGuard(obj, guard)
    % setGuard - configure guard condition of domain
    %
    % Copyright 2014 Texas A&M University AMBER Lab
    % Author: Ayonga Hereid <ayonga@tamu.edu>
    
    % argument check
%     narginchk(2,2);
%     
%     obj.guardName = guard.name;
%     obj.guardDir  = guard.direction;
%     obj.guardThld = guard.threshold;
%     obj.guardType = guard.type;
    
    
%     TYPE = 2; % 2=file, 3=MEX
%     if strcmp(obj.guardType,'kinematics') 
%         % then there should be associated mex file exists
%         
%         % check existance of mex files
%         assert(exist(strcat('guard_',obj.domainName),'file')==TYPE,...
%             'The MEX file could not be found: %s\n',...
%             strcat('guard_',obj.domainName)); % 3 - Mex-file
%         assert(exist(strcat('Jguard_',obj.domainName),'file')==TYPE,...
%             'The MEX file could not be found: %s\n',...
%             strcat('Jguard_',obj.domainName)); % 3 - Mex-file
%         
%         obj.guardFunc = str2func(strcat('guard_',obj.domainName));
%         obj.guardJac = str2func(strcat('Jguard_',obj.domainName));
%     end
           


end