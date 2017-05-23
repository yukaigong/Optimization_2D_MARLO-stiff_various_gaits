gaits_type=2;
speed=0;
tgspeed=0.7;
ctspeed=0.2;

options.ipopt.tol                    = 1e-4;
options.ipopt.dual_inf_tol           = 1e2;
options.ipopt.constr_viol_tol        = 1e-6;
options.ipopt.compl_inf_tol          = 1e2;
constraint_bound                     = 5e-6;

ipopt_optimization;

animateStep(outputs)