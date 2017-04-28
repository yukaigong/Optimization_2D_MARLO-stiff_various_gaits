function [ prob ] = loadOptProblem( optName )
%loadOptProblem Loads an default optimization problem with a few set
%parameters

%% Read from .yaml file
parent_path = pwd;

config_file = fullfile(parent_path,'config',...
    strcat(optName,'.yaml'));

% check if the file exists
assert(exist(config_file,'file')==2,...
    'The configuration file could not be found.\n');

% if exists, read the content and return as a structure
optConfig = cell_to_matrix_scan(yaml_read_file(config_file));


%% Setup opt problem
prob = optProblem(optConfig);

prob = initializeDomains(prob,optConfig.domains);
% load domains
domains  = prob.domains;

    % setup variable boudaies
    for i=1:prob.nDomain
        domainNames = fields(optConfig.boundaries);
        assert(length(domainNames) == prob.nDomain, 'All domains do not have boundaries specified')
            
        boundary = optConfig.boundaries.(domainNames{i});
        
        domains{i}.minJointAngles = boundary.minJointAngles;
        domains{i}.maxJointAngles = boundary.maxJointAngles;
          
        domains{i}.minJointVelocities = boundary.minJointVelocities;
        domains{i}.maxJointVelocities = boundary.maxJointVelocities;
          
        domains{i}.minJointAccels = boundary.minJointAccel;
        domains{i}.maxJointAccels = boundary.maxJointAccel;
        
        domains{i}.minTorques = boundary.minTorqueLimit;
        domains{i}.maxTorques = boundary.maxTorqueLimit;
        
        domains{i}.minConstrForces = boundary.minConstrForces;
        domains{i}.maxConstrForces = boundary.maxConstrForces;
        
        domains{i}.minTimeInterval = boundary.minTimeInterval;
        domains{i}.maxTimeInterval = boundary.maxTimeInterval;
        
        domains{i}.minAlpha = boundary.minAlpha;
        domains{i}.maxAlpha = boundary.maxAlpha;
               
        domains{i}.minImpactForces = boundary.minImpactForces;
        domains{i}.maxImpactForces = boundary.maxImpactForces;
    end
        
    prob.domains = domains;
end

