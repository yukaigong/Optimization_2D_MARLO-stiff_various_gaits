%> @brief Convert body struct to a spatial_v2 model
%> The purose of this is to facility an ease of conversion between
%> spatial_v2 and RBDL via files read in by yaml
function [model] = body_struct_to_spatial_model(body_struct, gravity)

if nargin < 2
    gravity = -9.81;
end

% dim = body_struct.dim; % @todo Add in option for planar
bodies = body_struct.dofs;
NB = length(bodies);
blankc = cell(NB, 1);
blankm = zeros(NB, 1);

model = struct();
% Fields for spatial_v2
model.NB = NB;
model.parent = blankm;
model.jtype = blankc;
model.Xtree = blankc;
model.I = blankc;
model.mass = blankm;
model.gravity = [0, 0, gravity];

% Additional fields for kinematics
model.name = blankc;
model.axis_index = blankm;
model.is_revolute = blankm;
axes = 'xyz';

for i = 1:NB
    body = bodies(i);
    model.parent(i) = body.lambda;
    
    full_index = find(body.axis);
    assert(isscalar(full_index));
    axis_value = body.axis(full_index);
    if full_index <= 3
        % Rotation
        prefix = 'R';
        index = full_index;
        model.is_revolute(i) = true;
    else
        prefix = 'P';
        index = full_index - 3;
    end
    
    if axis_value < 0
        jtype = ['-', prefix, axes(index)];
    else
        jtype = [prefix, axes(index)];
    end
    
    model.jtype{i} = jtype;
    model.Xtree{i} = plux(body.E, body.r);
    model.I{i} = mcI(body.mass, body.com, body.inertia);
    model.mass(i) = body.mass;
    
    model.name{i} = body.name;
    model.axis_index(i) = index;
end

end
