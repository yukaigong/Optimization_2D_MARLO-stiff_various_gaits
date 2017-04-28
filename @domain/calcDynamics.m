function [dx, u, Fe] = calcDynamics(obj, t, x, model, controller, ref)
    % calcDynamics - Calculate the continuous dynamics of the domain using
    % spatial vector algebra
    %   dx = f(x) + g(x)u
    %
    % Input:
    %    * t   - time
    %    * x   - system states
    %    * a   - virtual constraints parameters
    %    * p   - phase variable parameters
    %    * domain - domain
    %    * ref - class contains calculation results
    %
    % Output:
    %    * dx  - first order derivative of system states
    %
    % Copyright 2014-2015 Texas A&M University AMBER Lab
    % Author: Ayonga Hereid <ayonga@tamu.edu>


    % Extract states to angles and velocities
    qe  = x(model.qeIndices);
    dqe = x(model.dqeIndices);

    % calculate vector fields
    [vfc, gfc] = calcVectorFields();


    % calculate actual outputs
    [y_a1, y_a2, Dy_a1, Dy_a2, DLfy_a2] = calcActualOutputs(obj, x, model);
    [y_d1, y_d2, Dy_d1, Dy_d2, DLfy_d2, phip, tau, dtau, dy_d2_tau] = ...
        calcDesiredOutputs(obj, t, x, model, controller);


    
    y1    = y_a1 - y_d1;
    y2    = y_a2 - y_d2;
    Dy1   = Dy_a1 - Dy_d1;
    Dy2   = Dy_a2 - Dy_d2;
    
    
    Lfy2  = Dy2*vfc;
    
    DLfy2 = DLfy_a2 - DLfy_d2;
    
    
    % controller
    ep = controller.ep;
    
    %% calculate control input
    switch controller.type
        case 'IO'
            % input-output feedback linearization
            % Calculate relative degree one outputs
  
            if isempty(y1) % no relative degree one output case
                % Human-inspired controller
                
                % Second Lie Derivatives
                LfLfy2 = DLfy2*vfc;
                LgLfy2 = DLfy2*gfc;

                A_mat  = LgLfy2;
                Lf_mat = LfLfy2;
                mu = -2 * ep * Lfy2 - ep^2 * y2;
            else
                % Human-inspired controller
                Lfy1 = Dy1*vfc;
                Lgy1 = Dy1*gfc;
                Lfy2 = Dy2*vfc;
                % Second Lie Derivatives
                LfLfy2 = DLfy2*vfc;
                LgLfy2 = DLfy2*gfc;

                A_mat  = [Lgy1; LgLfy2];
                Lf_mat = [Lfy1; LfLfy2];
                mu = [-ep*y1;
                    -2 * ep * Lfy2 - ep^2 * y2];
                
            end

            % feedback controller
            u = A_mat \ (- Lf_mat + mu);
            
            calc.A = A_mat;
            calc.Lf = Lf_mat;
            calc.mu = mu;
            
            
            % Calculate the dynamics
            dx = vfc + gfc * u;
            % Calculate constrained forces
            Fe = -XiInv \ (Jedot * dqe + Je * (De \ (Be * u - He)));
        case 'PD'
            % extract PD gains
            kp = diag(controller.kp*ones(model.nRotor,1));
            kd = diag(controller.kd*ones(model.nRotor,1));
            
            [qd, dqd, calc_ik] = calcInverseKinematics(obj, t, x, model, controller,...
                y_d1, y_d2, dy_d2_tau, tau, dtau, phip);
            
            % PD controller
            u = -kp*(qe(model.qrIndices) - qd(model.qrIndices)) - ...
                kd*(dqe(model.qrIndices) - dqd(model.qrIndices));

            % Calculate the dynamics
            dx = vfc + gfc * u;
            % Calculate constrained forces
            Fe = -XiInv \ (Jedot * dqe + Je * (De \ (Be * u - He)));
        case 'QP-CLF'
            Bbar = [Be, Je'];
            vf   = [
                dqe;
                - De \ He];
            
            gf = [
                zeros(size(Bbar));
                De \  Bbar];
            AeqLagrangec = Je * (De \ Bbar);
            beqLagrangec = -1 * (Jedot * dqe - Je * (De \ He));
            
            
            if isempty(y1) % no relative degree one output case
                % Human-inspired controller
                
                % Second Lie Derivatives
                LfLfy2 = DLfy2*vf;
                LgLfy2 = DLfy2*gf;

                Lf_gammac = LfLfy2;
                etac      = [y2;Lfy2];
                Abarc     = LgLfy2;
            else
                % Human-inspired controller
                Lfy1 = Dy1*vf;
                
                Lfy2 = Dy2*vf;
                % Second Lie Derivatives
                LfLfy2 = DLfy2*vf;
                
                Lgy1 = Dy1*gf;
                LgLfy2 = DLfy2*gf;
                
                Lf_gammac = [Lfy1;
                    LfLfy2];
                
                etac = [y1;
                    y2;
                    Lfy2];

                Abarc =[Lgy1;
                    LgLfy2];
                
            end
            
            % QP-CLF 
            [u_qp] = calcClfMultiLagrange(obj, controller, model, ...
                etac, Abarc, Lf_gammac, AeqLagrangec,beqLagrangec);
            
            u = u_qp(1:obj.nAct);
            Fe = u_qp(obj.nAct+1:end);
            
            dx = vfc + gfc*u;
    end

    
    
    if strcmpi(obj.outputs.type,'Time-Based')
        dx = dx([2:model.nDof+1, model.nDof+3:end]);
    end
    
    
    
    
    % log data
    if nargin > 5

        calc.t = t;
        calc.x = x;


        calc.qe   = qe;
        calc.dqe  = dqe;
        calc.ddqe = dx(model.dqeIndices);
        calc.qs   = calc.qe(model.qsIndices);
        calc.dqs   = calc.dqe(model.qsIndices);
        
        calc.Je = Je;
        calc.dJe = Jedot;
        calc.De = De;
        calc.He = He;
        calc.vh = vfc(model.dqeIndices);
        calc.gh = gfc(model.dqeIndices, :);

        calc.u    = u;
        uq        = Be * u;
        calc.uq   = uq;
        calc.Fe   = Fe;
        calc.hd   = obj.holConstrFunc(x);

        calc.param = struct(obj.params);

        calc.tau  = tau;
        calc.dtau = dtau;
        calc.phip = obj.phip(x);
        calc.vhip = obj.vhip(x);
        calc.pcom = pe_com_vec(x);
        % compute power
        power = sum(uq' * dqe);
        calc.p    = power;
        if isempty(y_a1)
            calc.ya1 = 0;
            calc.yd1 = 0;
            calc.y1  = 0;
        else
            calc.ya1 = y_a1;
            calc.y1  = y1;
            calc.yd1 = y_d1;
        end
        calc.ya2 = y_a2;
        calc.yd2 = y_d2;
        calc.y2  = y2;
        calc.Lfy2 = Lfy2;
        
        calc.Be = Be;
        
        calc.vlineSpec = {obj.vlineSpec};

        % inverse kinematics
        if strcmpi(controller.type,'PD')
            calc.Phi = calc_ik.Phi;
            calc.h_a = calc_ik.h_a;
            calc.h_d = calc_ik.h_d;
            calc.err_y = calc_ik.err_y;
            calc.z1 = calc_ik.z1;
            calc.z2 = calc_ik.z2;
            calc.qd = calc_ik.qd;
            calc.dqd = calc_ik.dqd;
            calc.Phi_cond = calc_ik.Phi_cond;
            
            calc.h_a_actual = calc_ik.h_a_actual;
            calc.h_d = calc_ik.h_d;
            calc.dh_a_final = calc_ik.dh_a_final;
            calc.dh_d = calc_ik.dh_d;
        end
        
        
        ref.h.calc = calc;
    end


    %% calcVectorFields- Calculate the vector fields based on current state
    function [vfc, gfc] = calcVectorFields()
        
        
        % compute dynamics matrices
        De = De_mat(x); % inertia matrix
        Ce = Ce_mat(x); % coriolis matrix
        Ge = Ge_vec(x); % gravity vector
        Fs = Fs_vec(x); % spring forces
        Fr = Fr_vec(x); % friction forces
        
        He = Ce*dqe + Ge + Fs + Fr;
        
        
        % Calculate holonomic constraints jacobians
        Je    = obj.holConstrJac(x);
        Jedot = obj.holConstrJacDot(x);
        
        
        Ie = eye(model.nDof);
        XiInv = Je * (De \ transpose(Je));
        
        
        % check the validity of the holonomic constraints
        tol = 1e-3;
        dhe = Je * dqe;
        do_warn = true;
        if do_warn && norm(dhe) > tol
            warning('holonomic constraint violated');
%             keyboard;
        end
        
        
        switch controller.type
            case {'IO','QP-CLF'}
                % use partially actuated mode, i.e., assumes only those joints
                % directly associated with virtual constraints will be actuated
                % instead of all rotorary joints
                Be  = Ie(:,obj.qaIndices);
            case 'PD'
                Be  = Ie(:,model.qrIndices);
        end
        
        if controller.useUnconstrainedDynamics
            % if use unconstrained dynamics (consider constraints forces as
            % control inputs, then there will be no constraints )
            if strcmpi(controller.type,'IO') || strcmpi(controller.type,'PD')
                error('Unconstrained dynamics could not be used with either IO or PD controller.\n')
            end
            
            Bbar = [Be, transpose(Je)];
            if strcmpi(obj.outputs.type,'State-Based')
                % if the desired outputs are state-based trajectory, then
                % do the traditional way to compute vector fields
                % f(x)
                vfc = [
                    dqe;
                    - De \ He];
                
                % g(x)
                gfc = [
                    zeros(size(Bbar));
                    De \  Bbar];
            else  % Time-Based
                % if the desired outputs are time-based trajectory
                
                % f(x)
                vfc = [
                    1;
                    dqe;
                    0;
                    - De \ He];
                
                % g(x)
                gfc = [
                    zeros(1,size(Bbar,2));
                    zeros(size(Bbar));
                    zeros(1,size(Bbar,2));
                    De \ Bbar];
            end
            
        else
            
            if strcmpi(obj.outputs.type,'State-Based')
                % if the desired outputs are state-based trajectory, then
                % do the traditional way to compute vector fields
                
                % f(x)
                vfc = [
                    dqe;
                    De \ ((transpose(Je) * (XiInv \ (transpose(transpose(De) \ transpose(Je)))) -...
                    Ie) * He - transpose(Je) * (XiInv \ Jedot * dqe))];
                
                
                % g(x)
                gfc = [
                    zeros(size(Be));
                    De \ (Ie - transpose(Je)* (XiInv \ (transpose(transpose(De) \ transpose(Je))))) * Be];
            else  % Time-Based
                % if the desired outputs are time-based trajectory
                
                
                % f(x)
                vfc = [
                    1;
                    dqe;
                    0;
                    De \ ((transpose(Je) * (XiInv \ (transpose(transpose(De) \ transpose(Je)))) -...
                    Ie) * He - transpose(Je) * (XiInv \ Jedot * dqe))];
                
                
                % g(x)
                gfc = [
                    zeros(1,size(Be,2));
                    zeros(size(Be));
                    zeros(1,size(Be,2));
                    De \ (Ie - transpose(Je)* (XiInv \ (transpose(transpose(De) \ transpose(Je))))) * Be];
            end
        end
        
    end
end

