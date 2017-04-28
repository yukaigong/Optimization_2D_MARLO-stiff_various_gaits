function [a, p0] = find_transition(domain, x0, xf, outputIndices, updateP0)
%FIND_TRANSITION - Calculate the transition parameters using motion
%transition
%   Detailed explanation goes here

    controller = domain.ctrlConfig;
    %     a = controller.a;
    p = controller.p;
    %     vhip = p(1);
    nParam = (controller.nParamsRD2-3)/controller.nOutputs;
    a = reshape(p(4:end),[controller.nOutputs,nParam]);
    
    p0 = controller.delta_phip(x0);
    pf = controller.delta_phip(xf);
    
    if updateP0
        p(2) = p0;
        p(1) = pf;
    else
        p(3) = p0;
    end
    
    pdot0 = controller.J_delta_phip(x0)*x0(domain.dqeIndices);
    pdotf = controller.J_delta_phip(xf)*xf(domain.dqeIndices);

    [~,~,ya0,Dya0,~] = controller.y_a(x0);
    [~,~,yaf,Dyaf,~] = controller.y_a(xf);
    %     ya0 = controller.y_a2(x0);
    %     yaf = controller.y_a2(xf);
    %controller.v_hip
    %     Dya0 = controller.Dy_a2(x0);
    %     Dyaf = controller.Dy_a2(xf);
    
    Lfya0 = Dya0(:,domain.qeIndices)*x0(domain.dqeIndices);
    Lfyaf = Dyaf(:,domain.qeIndices)*xf(domain.dqeIndices);
    Y = [ya0, yaf, Lfya0, Lfyaf];
    
    tau_0 = (p0 - p(2))/(p(1)-p(2));
    tau_f = (pf - p(2))/(p(1)-p(2));
    if pdot0 == 0
        dtau_0 = 1;
    else
        dtau_0 = pdot0/(p(1)-p(2));
    end;
    dtau_f = pdotf/(p(1)-p(2));
    for i = 1:numel(outputIndices)
        index = outputIndices(i);
        a_temp   = zeros(4,7);
        if abs(a(index,6)) < 1e-6
            a(index,6) = 10;
        end
        a_temp(:,[2 4 6]) = ones(4,1) * a(index,[2 4 6]);
        a_temp(:,[1 3 5 7]) = eye(4);
        
        
        
        
        yd0 = cwf_time(a_temp,tau_0);
        ydf = cwf_time(a_temp,tau_f);
        Lfyd0 = dcwf_time(a_temp, tau_0)*dtau_0;
        Lfydf = dcwf_time(a_temp, tau_f)*dtau_f;
        
        
        a_rem = [yd0'; ydf'; Lfyd0'; Lfydf']\Y(index,:)';
        a(index,[1 3 5 7]) = a_rem;
    end
        
    p(4:end) = a(:);
    domain.ctrlConfig.p = p;
end

