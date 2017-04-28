setup;



optName = 'opt_2DWalking'

[opt,model] = loadOptProblem(optName);

opt.options


% add optimization variables
opt = configureOptVariables(opt);

% generate optimization boundaries
opt = genBoundaries(opt);

% generate initial guess
% opt = generateZ0(opt,calcs_steps, model);
opt = generateZ0(opt);
% checkZ0(opt);
%d
% add constraints
opt = configureConstraints(opt);
%
% add cost function
opt = configureObjective(opt);


J = IpoptJacobianStructure(opt.dimsConstr, ...
    opt.constrRows, opt.constrCols, opt.nOptVar);
density = 100*nnz(J)/numel(J);
spy(J);
hold all
% axis equal
plot([0,opt.nOptVar],[opt.domains{1}.dimsConstr,opt.domains{1}.dimsConstr]);
hold on;
plot([opt.domains{1}.nOptVars,opt.domains{1}.nOptVars],[0,opt.dimsConstr])
hold on;

% n1 = opt.domains{2}.optVarIndices.t(11);
% n2 = opt.domains{2}.optVarIndices.h(11,2);
% m1 = opt.domains{2}.constrArray(121).c_index(1);
% m2 = opt.domains{2}.constrArray(157).c_index(end);
n1 = 4374;
n2 = 4871;
m1 = 4859;
m2 = 5517;
plot([n1 n2],[m1 m1],'r'); hold on
plot([n1 n2],[m2 m2],'r'); hold on
plot([n1 n1],[m1 m2],'r'); hold on
plot([n2 n2],[m1 m2],'r'); hold on

h = gca;
h.XTick = [];
h.YTick = [];
xlabel('')


figure(3)
J_f = full(J);
J_n = sparse(J_f(m1:m2,n1:n2));

spy(J_n)

h = gca;
h.XTick = [];
h.YTick = [];
% title(['Maximum Density = ',num2str(density),'%'])
% xlabel('Variables')
% ylabel('Constraints')
% h = gca;
% title_pos = get(h.Title,'Position')
% set(h.Title,'Position',[title_pos(1) 10600 0])
% xlabel_pos = get(h.XLabel,'Position')
% set(h.XLabel,'Position',[xlabel_pos(1) -460 0])



