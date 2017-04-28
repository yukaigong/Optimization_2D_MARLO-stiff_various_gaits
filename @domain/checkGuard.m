function [value, isterminal, direction] = checkGuard(obj, t, x, model, controller, ref)
% GUARDCONDITION- Detect the guard condition (event trigger)
%    h(x) = 0
%   
% 
% Input: 
%    * domain - domain object
%    * t   - time
%    * x   - system states
%    * model - robot model object
% Copyright 2014-2015 Texas A&M University AMBER Lab
% Author: Ayonga Hereid <ayonga@tamu.edu>

    % substract 'q' from 'x' 
    %     qe = x(model.qeIndices);
    %     dqe = x(model.dqeIndices);

    direction  = obj.guardDir;
    isterminal = 1;
    
    
    switch obj.guardType
        case 'kinematics'
            % the function for kinematics guard function will be provided
            % by mathematica-generated MEX file.
            value = obj.guardFunc(x) - obj.guardThld;
        case 'forces'
            % find the index of the force in the constraints forces
            index = find(strcmpi(obj.holConstrName, obj.guardName));
            
            % check if the guard name is a valid constraint name
            assert(~isempty(index),...
                'Force-based guard condition could not be found in the constraints list: %s\n',...
                obj.guardName);
            % compute constraints forces 
            [~, ~, Fe] = calcDynamics(obj, t, x, model, controller);
            
            % compute the guard function
            value = Fe(index) - obj.guardThld;
        case 'time'
            % compute guard condition from the time signal
            value = t - obj.guardThld;
            assert(direction == 1,...
                'Time is monotonically increasing, therefore the direction should be positive.\n');
        case 'phase'
            % compute the current phase variable 'tau'
            tau = obj.tau(x, obj.controller.p);
            
            value = tau - obj.guardThld;
            
            assert(direction == 1,...
                'Tau is monotonically increasing, therefore the direction should be positive.\n');
    end
    
    
end