%> @brief HandCKinematic Extension to HandC to return kinematics as well
%> @author Roy Featherstone
%> @author Eric Cousineau <eacousineau@gmail.com>
function  [H, C, Xbase, S] = HandC_kinematic(model, q, qd, f_ext)

a_grav = get_gravity(model);

for i = 1:model.NB
  [ XJ, S{i} ] = jcalc( model.jtype{i}, q(i) );
  vJ = S{i}*qd(i);
  Xup{i} = XJ * model.Xtree{i};
  if model.parent(i) == 0
    v{i} = vJ;
    avp{i} = Xup{i} * -a_grav;
    Xbase{i} = Xup{i};
  else
    v{i} = Xup{i}*v{model.parent(i)} + vJ;
    avp{i} = Xup{i}*avp{model.parent(i)} + crm(v{i})*vJ;
    Xbase{i} = Xup{i} * Xbase{model.parent(i)};
  end
  fvp{i} = model.I{i}*avp{i} + crf(v{i})*model.I{i}*v{i};
end

if nargin == 4
  fvp = apply_external_forces( model.parent, Xup, fvp, f_ext );
end

for i = model.NB:-1:1
  C(i,1) = S{i}' * fvp{i};
  if model.parent(i) ~= 0
    fvp{model.parent(i)} = fvp{model.parent(i)} + Xup{i}'*fvp{i};
  end
end

IC = model.I;				% composite inertia calculation

for i = model.NB:-1:1
  if model.parent(i) ~= 0
    IC{model.parent(i)} = IC{model.parent(i)} + Xup{i}'*IC{i}*Xup{i};
  end
end

H = zeros(model.NB);

for i = 1:model.NB
  fh = IC{i} * S{i};
  H(i,i) = S{i}' * fh;
  j = i;
  while model.parent(j) > 0
    fh = Xup{j}' * fh;
    j = model.parent(j);
    H(i,j) = S{j}' * fh;
    H(j,i) = H(i,j);
  end
end
