%% one step
% for i=-15:1:15
%     speed=i/10
%     gaits_type=1;
%     ipopt_optimization;
%     save(['opt_result\avg_type1_' num2str(i) 'dms'],'outputs','x','info')
% 
% end
%% three step
for i=-12:1:12
    speed=i/10
    gaits_type=3;
    ipopt_optimization;
    save(['opt_result\avg_type3_' num2str(i) 'dms'],'outputs','x','info')
end