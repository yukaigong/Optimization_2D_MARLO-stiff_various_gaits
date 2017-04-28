function [updated_params,calcs_steps] = updateParameters(obj, raw_params, model)
% updateParameters - Update virtual constraint parameters in the case of
% bad convergence using motion transition, and export the updated
% parameters


    % if the optimization gait is non-periodic, then this method won't work
    assert(obj.options.PeriodicGait);
    
    %% load domain configuration
    parent_path = fileparts(pwd);
    behaviorName = obj.exports.behavior;
    config_file = fullfile(parent_path,'config','behavior',...
        strcat(behaviorName,'.yaml'));
        % check if the file exists
    assert(exist(config_file,'file')==2,...
        'The configuration file could not be found.\n');
    
    % if exists, read the content and return as a structure
    behaviorConfig = cell_to_matrix_scan(yaml_read_file(config_file));
    controller = behaviorConfig.controller;
    % setup controller for the simulation
    controller.type = 'IO';
    controller.ep = 10;
    controller.updatePhaseRange = true;
    updateParameters = true;
    
    nExDomains = numel(obj.exports.domains);
    ex_domains = cell(nExDomains,1);

    for i=1:nExDomains
        % create simulation ex_domains
        ex_domains{i} = domain(obj.exports.domains(i).target,i,model);
        
              
        
        % setup parameters to raw parameters
        ex_domains{i} = setParameters(ex_domains{i},raw_params{i});
        
        % compute p0 and pdot0
        x_plus = ex_domains{i}.params.x_plus;
        p0 = ex_domains{i}.deltaphip(x_plus);
        pdot0 = ex_domains{i}.Jdeltaphip(x_plus)*x_plus(model.dqeIndices);
        
        ex_domains{i} = setParameters(ex_domains{i},'p0',p0,'pdot0',pdot0);
    end
    
    for i=1:nExDomains
        
        % setup next domain
        if i==nExDomains
            next_domain = ex_domains{1};
            ex_domains{i} = setNextDomain(ex_domains{i},next_domain);
        else
            next_domain = ex_domains{i+1};
            ex_domains{i} = setNextDomain(ex_domains{i},next_domain);
        end
    end
    
    nSteps = 10;
    x0     = ex_domains{1}.params.x_plus;
    x_minus = ex_domains{end}.params.x_minus;
    ref = Ref();
    ref.h = struct();
    ref.h.calcs = {};
    ref.h.calc = {};
    t_plus = 0;
    
    calcs_steps = cell(nExDomains, nSteps);
    transition_tests = cell(nExDomains, nSteps);
    % simulate the system to update parameters
    for k = 1:nSteps
        tic
        for j = 1:nExDomains
            fprintf('{step: %d, domain: %d, t_plus: %g}\n', k, j, t_plus);
            if obj.options.UseSameParameters
                if j ~= 1 % not the first domain in a step
                    ex_domains{j}.params.p = ex_domains{j-1}.params.p;
                    ex_domains{j}.params.v = ex_domains{j-1}.params.v;
                end
            end
            current_domain = ex_domains{j};
            
            current_domain = setParameters(current_domain,'x_plus',x0);
            if obj.options.UseSameParameters
                x_update_target = blkdiag(model.footSwappingMatrix,model.footSwappingMatrix)*x_minus;
            else
                x_update_target = current_domain.params.x_minus;
            end
            if updateParameters
                if obj.options.TimeBased
                    [current_domain,transition_tests{j,k}] = calcMotionTransition(current_domain, model, controller, ...
                        current_domain.params.p(2), x0, current_domain.params.p(1), x_update_target);
                else
                    [current_domain,transition_tests{j,k}] = calcMotionTransition(current_domain, model, controller, ...
                        0, x0, 1, x_update_target);
                end
            end
           
            ref.h.calcs = {};
            ref.h.calc = {};
            ref.h.stepNo = k;
            
            odeopts = odeset('MaxStep', 1e-2, 'OutputFcn', @outputfcn, ...
                'Events', @(t, x, ref) checkGuard(current_domain, t, x, model, controller, ref),...
                'RelTol',1e-6,'AbsTol',1e-6);
            
            
            sol = ode113(@(t, x, ref) calcDynamics(current_domain, t, x, model, controller, ref), ...
                [0, 3], x0, odeopts, ref);
            
            % Compute reset map at the guard
            t_f = sol.x(end);
            x_f = sol.y(:,end);
            x0  = calcResetMap(current_domain, model, x_f);
            
            
            
            % log the simulation data
            calcDynamics(current_domain, t_f, x_f, model, controller, ref);
            ref.h.calcs{end} = ref.h.calc;
            
            % Store calculations
            calcs = horzcat_fields(cell2mat(ref.h.calcs));
            calcs.t_rel = calcs.t;
            calcs.t = t_plus + calcs.t;
            calcs.J = x_f(end);
            calcs_steps{j,k} = calcs;
            
            % Reset for next integration
            t_plus = t_plus + t_f;
            if obj.options.UseSameParameters
                if current_domain.indexInStep == current_domain.numDomainsInStep
                    x_minus = x_f;
                end
            else
                x_minus = x_f;
            end
            %         keyboard;
            current_domain = setParameters(current_domain,'x_minus',x_f);
            if obj.options.TimeBased
                current_domain.params.p(1) = t_f;
            end
            ex_domains{j} = current_domain;
        end
        toc
        
        
    end
    
    plotOutputs(ex_domains,calcs_steps,false);
    plotPhasePortraits(model,calcs_steps,false);
    plotZMP_new(ex_domains,model,calcs_steps,false);
    plotStates(model,calcs_steps,false);
    plotTorques(model,calcs_steps,false);
    keyboard;
    %     anim_plot_3D(model,calcs_steps,false,100);
    %     keyboard;
    % save and export the updated parameters
    
    updated_params   = cell(nExDomains,1);
    for i=1:nExDomains
        updated_params{i} = ex_domains{i}.params;
        updated_params{i}.name = ex_domains{i}.domainName;
    end
        
        
    
    
end