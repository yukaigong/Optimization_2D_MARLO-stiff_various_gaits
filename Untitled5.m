input_avg=[];
output_alpha=[];

switch gaits_type
    case 1
for i = -12:1:12
    load(['opt_result\avg_type1_' num2str(i) 'dms'])
%     if info.status == 0
        input_avg=[input_avg, i/10];
        output_alpha=[output_alpha, outputs{1}.a(1,:)'];
%     end
end
    case 2

x=input_avg;
re_output_alpha=zeros(size(output_alpha));
for i = 1:4
    index_1=(i-1)+[1 5 9 13 17 21];
    index_2=(i-1)*6+[1 2 3 4 5 6];
    re_output_alpha(index_2,:)=output_alpha(index_1,:);
end

for i = 1:4
    t = re_output_alpha(1+6*(i-1):6+6*(i-1),:);
    nntraining_periodic;
    
    for j = 1:6
        subplot(4,6,(i-1)*6+j)
        scatter(x,t(j,:))
        hold on;
        plot(x_line,y_line(j,:),'LineWidth',1);
        hold off;
    end
end
    