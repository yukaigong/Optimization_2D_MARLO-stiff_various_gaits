function domains = configureSystem(projName, v, a_mat, p0, pf, pm)
%CONFIGURESYSTEM - Configure the structure of the hybrid system for DURUS
%   'sys' consists of two steps (left stance and right stance) and each
%   step consists either one (single support) or two (double and single support)
%   domain. Steps that has more domains will be introduced later.



%% Step 1
rdsConfigFile =  ['../config/',projName,'/domain_right_double.yaml'];
rssConfigFile =  ['../config/',projName,'/domain_right_single.yaml'];
ldsConfigFile =  ['../config/',projName,'/domain_left_double.yaml'];
lssConfigFile =  ['../config/',projName,'/domain_left_single.yaml'];

rdsDomain = hybridDomain(rdsConfigFile);
rssDomain = hybridDomain(rssConfigFile);
ldsDomain = hybridDomain(ldsConfigFile);
lssDomain = hybridDomain(lssConfigFile);




a_right_double = a_mat([1:6],:); 
rdsDomain.ctrlConfig.p = [pf;p0;pm;a_right_double(:)];
rdsDomain.ctrlConfig.v = v;
rdsDomain.ctrlConfig.is_first_in_step = true;
rdsDomain.ctrlConfig.is_last_in_step = false;
% #heck:: for double support domains, only the waist and the stance leg
% joints are actuated
rightDSIndices = [rdsDomain.qWaistIndices
    rdsDomain.qLLegIndices([4])
    rdsDomain.qRLegIndices([3 4 5])];
rdsDomain.setActJoints(rightDSIndices);
leftDSIndices = [ldsDomain.qWaistIndices
    ldsDomain.qLLegIndices([3 4 5])
    ldsDomain.qRLegIndices([4])];
ldsDomain.setActJoints(leftDSIndices);


rightSSIndices = [rssDomain.qWaistIndices
    rssDomain.qLLegIndices(1:end-1);
    rssDomain.qRLegIndices([3 4 5])];
rssDomain.setActJoints(rightSSIndices);
leftSSIndices = [lssDomain.qWaistIndices    
    lssDomain.qLLegIndices([3 4 5]);
    lssDomain.qRLegIndices(1:end-1)];
lssDomain.setActJoints(leftSSIndices);


% #heck:: for double support domains, there is no impact at the end of the
% domain (spring lock)
rdsDomain.hasImpact = false;
ldsDomain.hasImpact = false;


a_right_single = a_mat;
rssDomain.ctrlConfig.p = [pf;p0;pm;a_right_single(:)];
rssDomain.ctrlConfig.v = v;
rssDomain.ctrlConfig.is_first_in_step = false;
rssDomain.ctrlConfig.is_last_in_step = true;

rollOutputs = strfind(rssDomain.ctrlConfig.outputName_RD2,'Roll');
rollOutputIndices = find(~cellfun(@isempty,rollOutputs));
yawOutputs = strfind(rssDomain.ctrlConfig.outputName_RD2,'Yaw');
yawOutputIndices = find(~cellfun(@isempty,yawOutputs));
roll_yaw_outputs_indices = sort([rollOutputIndices;yawOutputIndices]);




a_left_double  = a_mat([1:6],:);
if size(a_mat,2) == 7 %ECWF
    a_left_double(roll_yaw_outputs_indices(1:2),:) = ...
        -a_left_double(roll_yaw_outputs_indices(1:2),:);
else %CWF
    a_left_double(roll_yaw_outputs_indices(1:2),:) = ...
        -a_left_double(roll_yaw_outputs_indices(1:2),:);
end
ldsDomain.ctrlConfig.p = [pf;p0;pm;a_left_double(:)];
ldsDomain.ctrlConfig.v = v;
ldsDomain.ctrlConfig.is_first_in_step = true;
ldsDomain.ctrlConfig.is_last_in_step = false;
a_left_single  = a_mat;
if size(a_mat,2) == 7 %ECWF
    a_left_single(roll_yaw_outputs_indices,:) = ...
        -a_left_single(roll_yaw_outputs_indices,:);
else
    a_left_single(roll_yaw_outputs_indices,:) = ...
        -a_left_single(roll_yaw_outputs_indices,:);
end
lssDomain.ctrlConfig.p = [pf;p0;pm;a_left_single(:)];
lssDomain.ctrlConfig.v = v;
lssDomain.ctrlConfig.is_first_in_step = false;
lssDomain.ctrlConfig.is_last_in_step = true;
domains = {rdsDomain,rssDomain,ldsDomain,lssDomain};


%% discrete regulator
for i=1:numel(domains)
    domains{i}.dlp.desired_velocity = 0.35;
    domains{i}.dlp.pushoff_gain     = -1.4;
    
    domains{i}.dlp.deltaQ_current = 0;
    
    domains{i}.dlp.deltaQ_prev = 0;
    
    domains{i}.dlp.tau_takeoff = 0;
end

end

