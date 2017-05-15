input_avg=[];
output_alpha=[];
for i = -12:1:12
    load(['opt_result\avg_type1_' num2str(i) 'dms'])
%     if info.status == 0
        input_avg=[input_avg, i/10];
        output_alpha=[output_alpha, outputs{1}.a(1,:)'];
%     end
end
