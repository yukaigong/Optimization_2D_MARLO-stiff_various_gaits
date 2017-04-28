function [model] = planar_body_struct_to_spatial_model(body_struct)

% dim = body_struct.dim; % @todo Add in option for planar
bodies = body_struct.bodies;
NB = length(bodies);
blankc = cell(NB, 1);
blankm = zeros(NB, 1);

model = struct();
% Fields for spatial_v2
model.NB = NB;
model.parent = [bodies.lambda];
model.jtype = blankc;
model.Xtree = blankc;
model.I = blankc;
model.gravity = [0, -9.81];

% Additional fields
%> @todo Put this in a substructure so that it's clearly separated?
model.mass = [bodies.mass];
model.name = {bodies.name};
model.is_revolute = blankm;
axes = 'xy';

for i = 1:NB
    body = bodies(i);
    model.parent(i) = body.lambda;
    
    full_index = find(body.axis);
    assert(isscalar(full_index));
    axis_value = body.axis(full_index);
    if full_index == 1
        % Rotation
        index = full_index;
        model.is_revolute(i) = true;
        jtype = 'r';
    else
        index = full_index - 1;
        jtype = ['p', axes(index)];
    end
    
    if axis_value < 0
        jtype = ['-', jtype];
    end
    
    model.jtype{i} = jtype;
    model.Xtree{i} = plnr(0, body.offset);
    model.I{i} = mcI(body.mass, body.com, body.I);
    model.mass(i) = body.mass;
    
    model.name{i} = body.name;
    model.axis_index(i) = index;
end

% Input positions
model.positions = body_struct.positions;

end
