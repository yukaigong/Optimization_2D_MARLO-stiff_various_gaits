function [obj, test] = calcMotionTransition(obj, model, controller, t0, x0, tf, xf)
    % calcMotionTransition - calculate transition matrix through motion
    % transition
    
    % extract old parameters
    params   = obj.params;
    a = params.a;
    p = params.p;
    v = params.v;
    nOutputs = obj.nOutputs;
    % number of parameters for each output
    nParams  = numel(a)/nOutputs;
    
    % write in the matrix form
    a_mat = reshape(a,[nOutputs,nParams]);
    
    params_init = struct(params);
    
    % compute actual outputs
    if ~isempty(obj.outputs.actual.degreeOneOutput)
        ya10 = obj.ya1(x0);
    end
    ya0 = obj.ya2(x0);
    yaf = obj.ya2(xf);
    Dya0 = obj.Dya2(x0);    
    Dyaf = obj.Dya2(xf);
    Lfya0 = Dya0(:,model.qeIndices)*x0(model.dqeIndices);
    Lfyaf = Dyaf(:,model.qeIndices)*xf(model.dqeIndices);
    Y = [ya0, yaf, Lfya0, Lfyaf];
    
    velocity_tolerance = 1e-6;
    
    % compute phase variable
    if strcmpi(obj.outputs.type,'Time-Based')
        tau_0 = t0;
        tau_f = tf;
        dtau_0 = 1;
        dtau_f = 1;
        
        if controller.updatePhaseRange
            p(1) = tf;
            p(obj.indexInStep+1) = t0;
        end
        p0 = obj.deltaphip(x0);
        pf = obj.deltaphip(xf);
    else
        p0 = obj.deltaphip(x0);
        pf = obj.deltaphip(xf);
        if controller.updatePhaseRange
            p(1) = pf;
            p(obj.indexInStep+1) = p0;
        end
        tau_0   = obj.tau(x0,p);   % phase variable
        dtau_0  = obj.dtau(x0,p);  % time derivative of phase variable
        
        tau_f   = obj.tau(xf,p);   % phase variable
        dtau_f  = obj.dtau(xf,p);  % time derivative of phase variable
        
        if abs(dtau_0) < velocity_tolerance
            % if dtau_0 equals to zero (starting behavior), then set it to
            % 1
            dtau_0 = 1;
        end
        if abs(dtau_f) < velocity_tolerance
            % if dtau_f equals to zero (stopping behavior), then set it to
            % 1
            dtau_f = 1;
        end
    end
    
    switch obj.outputs.desired.degreeOneOutput
        case 'MinJerk'
            v(2) = ya10;
    end
    
    % based on function type, choose which parameters to reserve, and which
    % to update
    switch obj.outputs.desired.degreeTwoOutput 
        case 'Bezier5thOrder'
            reserveParams = [3];
            updateParams  = [1 2 4 5];
            
            updateOutputIndices  = obj.outputs.actual.nonZeroOutputIndices;
            % compute motion trasition
            for i = 1:numel(updateOutputIndices)
                index = updateOutputIndices(i);
                a_temp   = zeros(4,nParams);
                a_rest   = zeros(1,nParams);
                
                
                a_rest(:,reserveParams) = a_mat(index,reserveParams);
                a_temp(:,updateParams) = eye(4);
                
                
                yd0r = bezier(a_rest,tau_0);
                ydfr = bezier(a_rest,tau_f);
                Lfyd0r = dbezier(a_rest,tau_0)*dtau_0;
                Lfydfr = dbezier(a_rest,tau_f)*dtau_f;
                
                yd0 = bezier(a_temp,tau_0);
                ydf = bezier(a_temp,tau_f);
                Lfyd0 = dbezier(a_temp,tau_0)*dtau_0;
                Lfydf = dbezier(a_temp,tau_f)*dtau_f;
                
                a_bias = [yd0r;ydfr;Lfyd0r;Lfydfr];
                Phi_rem = [yd0'; ydf'; Lfyd0'; Lfydf'];
                
                a_rem = Phi_rem\(Y(index,:)'-a_bias);
                a_mat(index,updateParams) = a_rem;
                
            end
            
            if nargout >= 2
                input = struct();
                input.domain_name = obj.domainName;
                input.x0 = x0;
                input.xf = xf;
                input.output_indices = updateOutputIndices;
                input.index_in_step = obj.indexInStep;
                input.velocity_tolerance = velocity_tolerance;
                input.params = params_init;
                output = struct();
                output.params = [];
                output.p0 = p0;
                output.pf = pf;
                output.tau0 = tau_0;
                output.tauf = tau_f;
                output.dtau0 = dtau_0;
                output.dtauf = dtau_f;
                output.Y = Y;
                % Last values
                output.a_bias = a_bias;
                output.Phi_rem = Phi_rem;
                test = struct();
                test.input = input;
                test.output = output;
            end
        case 'Bezier7thOrder'
            reserveParams = [3 4 5];
            updateParams  = [1 2 6 7];
            
            updateOutputIndices  = obj.outputs.actual.nonZeroOutputIndices;
            % compute motion trasition
            for i = 1:numel(updateOutputIndices)
                index = updateOutputIndices(i);
                a_temp   = zeros(4,nParams);
                a_rest   = zeros(1,nParams);
                
                
                a_rest(:,reserveParams) = a_mat(index,reserveParams);
                a_temp(:,updateParams) = eye(4);
                
                
                yd0r = bezier(a_rest,tau_0);
                ydfr = bezier(a_rest,tau_f);
                Lfyd0r = dbezier(a_rest,tau_0)*dtau_0;
                Lfydfr = dbezier(a_rest,tau_f)*dtau_f;
                
                yd0 = bezier(a_temp,tau_0);
                ydf = bezier(a_temp,tau_f);
                Lfyd0 = dbezier(a_temp,tau_0)*dtau_0;
                Lfydf = dbezier(a_temp,tau_f)*dtau_f;
                
                a_bias = [yd0r;ydfr;Lfyd0r;Lfydfr];
                Phi_rem = [yd0'; ydf'; Lfyd0'; Lfydf'];
                
                a_rem = Phi_rem\(Y(index,:)'-a_bias);
                a_mat(index,updateParams) = a_rem;
                
            end
            
            if nargout >= 2
                input = struct();
                input.domain_name = obj.domainName;
                input.x0 = x0;
                input.xf = xf;
                input.output_indices = updateOutputIndices;
                input.index_in_step = obj.indexInStep;
                input.velocity_tolerance = velocity_tolerance;
                input.params = params_init;
                output = struct();
                output.params = [];
                output.p0 = p0;
                output.pf = pf;
                output.tau0 = tau_0;
                output.tauf = tau_f;
                output.dtau0 = dtau_0;
                output.dtauf = dtau_f;
                output.Y = Y;
                % Last values
                output.a_bias = a_bias;
                output.Phi_rem = Phi_rem;
                test = struct();
                test.input = input;
                test.output = output;
            end
        case 'ECWF'
            reserveParams = [2 4 6];
            updateParams  = [1 3 5 7];
            
            updateOutputIndices  = obj.outputs.actual.nonZeroOutputIndices;
            % compute motion trasition
            for i = 1:numel(updateOutputIndices)
                index = updateOutputIndices(i);
                a_temp   = zeros(4,nParams);
                if abs(a_mat(index,6)) < 1e-6
                    a_mat(index,6) = 10;
                end                
                a_temp(:,reserveParams) = ones(4,1) * a_mat(index,reserveParams);
                a_temp(:,updateParams) = eye(4);
                
                
                
                yd0 = cwf_time(a_temp,tau_0);
                ydf = cwf_time(a_temp,tau_f);
                Lfyd0 = dcwf_time(a_temp,tau_0)*dtau_0;
                Lfydf = dcwf_time(a_temp,tau_f)*dtau_f;
                
                Phi_rem = [yd0'; ydf'; Lfyd0'; Lfydf'];
                
                a_rem = Phi_rem\(Y(index,:)');
                a_mat(index,updateParams) = a_rem;
                
            end
            
            if nargout >= 2
                % No unittest info
                test = struct();
            end
        otherwise
            error('Unable to perform motion transition for this specified desired output type');
    end
    
    
    % update relative degree one output parameters in case of MinJerk
    % function
    %     if controller.updateRD1Parameters
    %         switch obj.outputs.desired.degreeOneOutput
    %             case 'MinJerk'
    %
    %             otherwise
    %                 error('Unable to perform motion transition for this specified desired output type');
    %         end
    %     end
    
    % update parameters 
    
    
    if controller.updatePhaseRange
        obj = setParameters(obj, 'p', p, 'a', a_mat, 'v', v);
    else
        obj = setParameters(obj, 'a', a_mat, 'v', v);
    end
    
    if nargout >= 2
        test.output.params = obj.params;
    end
    
end