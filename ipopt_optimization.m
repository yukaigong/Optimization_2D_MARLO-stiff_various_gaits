%% Direct Collocation Based Optimization
setup;

% gaits_type=1; % type 1 is periodic;
%               % type 2 is three step transient
%               % type 3 is three step periodic
% speed=-1.2;
% if gaits_type==1
%     optName = 'opt_2DWalking'
% end
% 
% if gaits_type==2
%     optName = 'opt_2DWalking_transient';
% end
switch gaits_type
    case 1
        optName = 'opt_2DWalking'
    case 2
        optName = 'opt_2DWalking_transient'
    case 3
        optName = 'opt_2Dwalking_transient';
    otherwise
        disp('gaits type wrong')
end

opt = loadOptProblem(optName);
opt.options

% add optimization variables
opt = configureOptVariables(opt);

% generate optimization boundaries
opt = genBoundaries(opt);

% generate initial guess
opt = generateZ0(opt);

% add constraints
opt = configureConstraints(opt,gaits_type,speed,tgspeed,ctspeed);

% add cost function
opt = configureObjective(opt);

% Get Initial Condition
x0 = opt.Z0;
old = load(['x_gaits_type=' num2str(gaits_type)]);
x0 = old.x;

%% Solve Optimization Problem
debugMode = false;

% % IPOPT options
options.lb = opt.lb;
options.ub = opt.ub;
options.cl = opt.cl;
options.cu = opt.cu;

options.ipopt.mu_strategy      = 'adaptive';
options.ipopt.max_iter         = 2000;
options.ipopt.tol              = 1e-4;
%     disp(['max iterations = ', num2str(options.ipopt.max_iter)])
%     disp(['tolerance = ', num2str(options.ipopt.tol)])

% options.ipopt.dual_inf_tol           = 1e2;
% options.ipopt.constr_viol_tol        = 1e-13;
% options.ipopt.compl_inf_tol          = 1e2;

%options.acceptable_tol  = 1e3;
%options.acceptable_compl_inf_tol    = 1e0;

% check_derivatives_for_naninf  = 'yes';

options.ipopt.hessian_approximation = 'limited-memory';
options.ipopt.limited_memory_update_type = 'bfgs';  % {bfgs}, sr
options.ipopt.limited_memory_max_history = 50;  % {6}
options.ipopt.recalc_y = 'yes';
%options.ipopt.recalc_y_feas_tol = 1e-3;

%options.ipopt.bound_relax_factor = 1e-3;
options.ipopt.linear_solver = 'ma57';
options.ipopt.fixed_variable_treatment = 'RELAX_BOUNDS';
% options.ipopt.derivative_test = 'first-order';
options.ipopt.point_perturbation_radius = 0;
options.ipopt.bound_push = 0.000001;

% options.ipopt.derivative_test_perturbation = 1e-8;
% options.ipopt.derivative_test_print_all = 'yes';
options.ipopt.ma57_automatic_scaling = 'no';
options.ipopt.linear_scaling_on_demand = 'no';
% The callback functions.
funcs.objective    = @(x)IpoptObjective(x, opt.costArray, opt.constrArray, debugMode);
funcs.constraints  = @(x)IpoptConstraints(x, opt.constrArray, opt.dimsConstr, debugMode);
funcs.gradient     = @(x)IpoptGradient(x, opt.costArray, ...
    opt.costRows, opt.costCols, opt.nOptVar, debugMode);
funcs.jacobian     = @(x)IpoptJacobian(x, opt.dimsConstr, ...
    opt.constrArray, opt.constrRows, opt.constrCols, opt.nOptVar, debugMode);
funcs.jacobianstructure = @()IpoptJacobianStructure(opt.dimsConstr, ...
    opt.constrRows, opt.constrCols, opt.nOptVar, debugMode);
% funcs.hessian           = @hessian;
% funcs.hessianstructure  = @hessianstructure;

% Start optimization
tic
[x, info] = ipopt(x0,funcs,options);
toc

[outputs] = getOptOutput(opt, x)
save(['x_gaits_type=' num2str(gaits_type)],'x')
% animateStep(outputs)



