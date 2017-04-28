function [qd, dqd, calc] = calcInverseKinematics(obj, t, x, model, controller, y_d1, y_d2, dy_d2, tau, dtau, phip)
    % calcInverseKinematics

    if controller.applyDiscreteRegulation
        % HUBICKI ADDITION
        min_knee_angle = 0.15;
        [s_reg_signal, ns_reg_signal] = regulation_signal(tau, obj.dlp, obj.indexInStep);
        
        y_d2(1) = y_d2(1) + s_reg_signal;
        if y_d2(1) < min_knee_angle
            y_d2(1) = min_knee_angle;
        end
        y_d2(6) = y_d2(6) + ns_reg_signal;
        if y_d2(6) < min_knee_angle
            y_d2(6) = min_knee_angle;
        end
        % END HUBICKI ADDITION
    end
    % Extract states to angles and velocities
    qe  = x(model.qeIndices);
    dqe = x(model.dqeIndices);


    %% IK Helpers
    params = obj.params;
    p0    = params.p0;
    pdot0 = params.pdot0;
    
    p     = params.p;
    ep    = controller.ep;
    vhip  = y_d1;
    if  ~isempty(obj.outputs.actual.degreeOneOutput)
        isUnderActuatedDomain = false;
    else
        isUnderActuatedDomain = true;
    end
    
    
    
    % desired kinematics outputs
    cur_constr = obj.holConstrFunc(x);
    %     if obj.indexInStep == 1
    %         cur_constr([3:6 9:15]) = zeros(11,1);
    %     else
    %         cur_constr([3:9]) = zeros(7,1);
    %     end
    %% determine phase variables
    if isUnderActuatedDomain
        
        h_d = [y_d2;
            cur_constr;
            qe(obj.qzIndices)];
        z1 = p(2) + (p(1) - p(2)) * tau;
        z2 = (p(1) - p(2)) * dtau;
    
    else
        if controller.use_phip_from_tau
            z1 = p(2) + (p(1) - p(2)) * tau;
            z2 = (p(1) - p(2)) * dtau;
        else
            z1 = vhip*t + p0 + ((1-exp(-t*ep))*pdot0 + (-1+exp(-t*ep))*vhip)/ep;
            z2 = vhip + exp(-t*ep)*(pdot0 - vhip);
        end
        
        h_d = [z1;
            y_d2;
            cur_constr;
            qe(obj.qzIndices)];
        
    end
    count = 1;
    tol_q = 1e-8;
    qd = qe;
    Ie = eye(model.nDof);
    Jz = Ie(obj.qzIndices,:);
    while true
        xd = [qd; dqe];
        y_a2 = obj.ya2(xd);
        Dy_a2 = obj.Dya2(xd);
        
        Jd    = obj.holConstrJac(xd);
        if isUnderActuatedDomain
            Phi = [Dy_a2(:,model.qeIndices);
                Jd;
                Jz
                ];
            
            h_a = [y_a2;
                obj.holConstrFunc(xd);
                qd(obj.qzIndices)];
        else
            Dy_a1 = obj.Dya1(xd);
            Phi = [Dy_a1(:,model.dqeIndices);
                Dy_a2(:,model.qeIndices);
                Jd;
                Jz
                ];
            
            h_a = [obj.deltaphip(xd);
                y_a2;
                obj.holConstrFunc(xd);
                qd(obj.qzIndices)];
        end
        
        err_y = h_a - h_d;
        
        if count == 1
            % Store first set of computations
            calc.Phi = Phi;
            calc.h_a = h_a;
            calc.h_d = h_d;
            calc.err_y = err_y;
        end
        
        if maxabs(err_y) <= tol_q
            break;
        end
        
        % otherwise update
        assert(rank(Phi) == model.nDof);
        assert(size(Phi, 1) == model.nDof);
        delta = Phi \ err_y;
        qd    = qd - delta;
        for k = [model.qbIndices(4:6);model.qrIndices]'
            if qd(k) > pi
                qd(k) = qd(k) - 2*pi;
            elseif qd(k) < -pi
                qd(k) = qd(k) + 2*pi;
            end
        end
        count = count+1;
        if count >= 1000
            keyboard;
        end
    end
    
    
    
    if isUnderActuatedDomain
        Dy_a2 = obj.Dya2([qd;dqe]);
        Jd    = obj.holConstrJac([qd;dqe]);
        
        Phi = [Dy_a2(:,model.qeIndices);
            Jd;
            Jz
            ];
    
    
        dh_d = [dy_d2;zeros(obj.nHolConstr,1);dqe(obj.qzIndices)];
        dqd = Phi\dh_d;
    else
        Dy_a1 = obj.Dya1([qd;dqe]);
        Dy_a2 = obj.Dya2([qd;dqe]);
        Jd    = obj.holConstrJac([qd;dqe]);
        
        Phi = [Dy_a1(:,model.dqeIndices);
            Dy_a2(:,model.qeIndices);
            Jd;
            Jz
            ];
    
    
        dh_d = [z2;dy_d2;zeros(obj.nHolConstr,1);dqe(obj.qzIndices)];
        dqd = Phi\dh_d;
    end
    calc.z1 = z1;
    calc.z2 = z2;
    calc.qd = qd;
    calc.dqd = dqd;
    calc.Phi_cond = cond(Phi);
    
    if isUnderActuatedDomain
        
        calc.h_a_actual = [y_a2;
            obj.holConstrFunc([qe;dqe]);
            qe(obj.qzIndices)];
    else
        calc.h_a_actual = [phip;
            y_a2;
            obj.holConstrFunc([qe;dqe]);
            qe(obj.qzIndices)];
    end
    calc.h_d = h_d;
    calc.dh_a_final = Phi * dqe;
    calc.dh_d = dh_d;
    
    
end