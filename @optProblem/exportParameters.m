function [param_name] = exportParameters(obj, x, model)

    if ~obj.options.OpenLoopController
        
        domain_names = cell(obj.nDomain,1);
        for i = 1:obj.nDomain
            domain_names{i} = obj.domains{i}.domainName;
        end
        
        export_behavior = obj.exports.behavior;
        export_domains  = obj.exports.domains;
        
        nExDomains = numel(export_domains);
        export_params   = cell(nExDomains,1);
        for i = 1:nExDomains
            cur = export_domains(i);
            source = obj.domains{strcmp(domain_names,cur.source)};
            target = cur.target;
            params = struct();
            av = x(source.optVarIndices.a(1,:));
            p  = x(source.optVarIndices.p(1,:));
            v  = x(source.optVarIndices.v(1,:));
            n_output = source.nOutputs;
            n_params = source.nParamRD2/n_output;
            a  = reshape(av,n_output,n_params);
            
            
            x_plus = ... 
                [x(source.optVarIndices.q(1,:))';
                x(source.optVarIndices.dq(1,:))'];
            x_minus = ... 
                [x(source.optVarIndices.q(end,:))';
                x(source.optVarIndices.dq(end,:))'];
            
            params.name = target;
            params.v = v;
            
            if obj.options.TimeBased
                p(1) = x(source.optVarIndices.t(end));
                p(2) = 0;
            end
            params.p = p;
            
            if isempty(cur.swapOutputIndices)
                params.a = a;
            else
                a(cur.swapOutputIndices,:) = - a(cur.swapOutputIndices,:);
                params.a = a;
            end
            
            
            if cur.swapFoot
                params.x_plus =...
                    blkdiag(model.footSwappingMatrix,model.footSwappingMatrix)*x_plus;
                params.x_minus = ...
                    blkdiag(model.footSwappingMatrix,model.footSwappingMatrix)*x_minus;
            else
                params.x_plus = x_plus;
                params.x_minus = x_minus;
            end
            
            export_params{i} = params;
            obj.domains{i}.params = params;
        end
        
        
        [calcs_opt] = extractResult(obj, x, model);
        
        
        export_data = struct();
        if obj.options.ExportUpdatedParameters
            [updated_params,calcs_steps] = updateParameters(obj,export_params,model);
            export_data.domain = vertcat(updated_params{:});
            export_data.gait_metrics = gait_metrics(obj.domains,model,calcs_steps(:,end));
        else
            export_data.domain = vertcat(export_params{:});
            export_data.gait_metrics = gait_metrics(obj.domains,model,calcs_opt);
            calcs_steps = [];
        end
            
            
        [~,timestr]= system(['date -Im | sed ','s#:#-#g']);
        
        export_result = ['../results/',export_behavior,'/','optResult_',strtrim(timestr),'.mat'];
        if ~exist(['../results/',export_behavior],'dir')
            mkdir(['../results/',export_behavior]);
        end
        save(export_result,'calcs_opt','model','calcs_steps');
        
        
        
        param_name = ['params_',strtrim(timestr)];
        export_name = ['../config/parameters/',export_behavior,'/',param_name,'.yaml'];
        if ~exist(['../config/parameters/',export_behavior],'dir')
            mkdir(['../config/parameters/',export_behavior]);
        end
        
        yaml_write_file(export_name,export_data);
    end

end