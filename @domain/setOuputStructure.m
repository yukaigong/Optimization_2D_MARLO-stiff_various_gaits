function obj = setOuputStructure(obj, outputs)
    % configureOuputStructure - configure output structure of domain
    %
    % Copyright 2014 Texas A&M University AMBER Lab
    % Author: Ayonga Hereid <ayonga@tamu.edu>
%     narginchk(3,3);
    
    % output structure from domain configuration
%     obj.outputs = outputs;   
    
%     TYPE = 2; % 2 = file, 3 = mex
    
    
%     obj.nOutputs = numel(outputs.actual.degreeTwoOutput);
    
%     funcs = {...
%         'ya2',...
%         'yd2',...
%         'Dya2',...
%         'DLfya2',...
%         'dyd2',...
%         'ddyd2',...
%         'deltaphip',...
%         'Jdeltaphip',...
%         'tau',...
%         'dtau',...
%         'Jtau',...
%         'Jdtau'};
    
%     funcs = {...
%         'ya2',...
%         'yd2',...
%         'Dya2',...
%         'dyd2',...
%         'tau',...
%         'dtau',...
%         'Jtau',...
%         'Jdtau'};
    
%     % if relative degree one output is defined, then add corresponding
%     % function handles
%     if ~isempty(obj.outputs.actual.degreeOneOutput) 
%         funcs = horzcat({'ya1','Dya1','yd1','dyd1'},funcs);
%         nOutputRD1 = 1;
%     else
%         nOutputRD1 = 0;
%     end
    
    
%     domainName = obj.domainName;
%     for i=1:numel(funcs)
%         % obtain file name
%         filename = strcat(funcs{i},'_',domainName);
%         % check if the mex file exists
%          assert(exist(filename,'file')==TYPE,...
%              'MEX file is not found: %s',filename);
%         % assign to domain fields
%         obj.(funcs{i}) = str2func(filename);
%     end
    
    % number of parameters
%     obj.nParamPhaseVar = obj.outputs.nParamPhaseVar; 
    
    
%     if ~isempty(obj.outputs.desired.degreeOneOutput)
%         type = obj.outputs.desired.degreeOneOutput;
%         obj.nParamRD1 = getNumberofParameters(type);
%     else
%         obj.nParamRD1 = 0;
%     end
    
%     type = obj.outputs.desired.degreeTwoOutput;
%     nParams = getNumberofParameters(type);
%     obj.nParamRD2 = nParams * obj.nOutputs;
    
    % set actuated joints
%      obj.nAct = numel(outputs.actuatedJoints);
%     assert(obj.nAct == nOutputRD1 + obj.nOutputs, ...
%         'The number of actuated joints is not equal to number of outputs');
    
%     obj.qaIndices = getJointIndices(model,outputs.actuatedJoints);
%     obj.dqaIndices = obj.qaIndices + model.nDof;
    

    % Commented out by Ross
    % set zero dynamics indices
%     obj.qzIndices = getJointIndices(model,outputs.zeroDynamics);
%     obj.dqzIndices = obj.qzIndices + model.nDof;
%     obj.nZero = numel(obj.qzIndices);
%     
    
%     function [nParams] = getNumberofParameters(type)
%         switch type
%             case 'Constant'
%                 nParams = 1;
%             case 'MinJerk'
%                 nParams = 3;
%             case 'CWF'
%                 nParams = 5;
%             case 'Bezier5thOrder'
%                 nParams = 5;
%             case 'Bezier6thOrder'
%                 nParams = 6;
%             case 'Bezier7thOrder'
%                 nParams = 7;
%             case 'ECWF'
%                 nParams = 7;
%             otherwise
%                 error('invalid function type.\n');
%         end
    
%     end
end