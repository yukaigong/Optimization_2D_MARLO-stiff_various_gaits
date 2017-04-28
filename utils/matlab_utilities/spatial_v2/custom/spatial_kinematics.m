%> @brief spatial_kinematics Extension to HandC to return kinematics as well
%> @note From amber.control.HandCKinematic
function  [Xbase, XJs, v] = spatial_kinematics(model, q, qd)

% TODO Get page references to these things, or look at RBDL
% Put in original software package, docs, license, etc
% ID p.96

% Is there a way to sneak jacobians in here? Or extract it from constrained
% dynamics?

NB = model.NB;

blank = cell(NB, 1);
S = blank;
XJs = blank;
Xup = blank; % What does 'up' mean?!
Xbase = blank; % base to body frame
v = blank;

% Compute Kinematics
for i = 1:model.NB
    [ XJ, S{i} ] = jcalc( model.jtype{i}, q(i) );
    vJ = S{i} * qd(i);
    % Body frame effect (XJ) orienting the base transform (Xtree)
    % Transforms parent frame -> current
    Xup{i} = XJ * model.Xtree{i};
    XJs{i} = XJ;
    
    lambda = model.parent(i);
    if lambda == 0
        v{i} = vJ;
        Xbase{i} = Xup{i};
    else
        v{i} = Xup{i} * v{lambda} + vJ;
        Xbase{i} = Xup{i} * Xbase{lambda};
    end
end

end
