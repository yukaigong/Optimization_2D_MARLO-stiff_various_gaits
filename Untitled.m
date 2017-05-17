input_avg=[];
output_alpha=[];
for i = -7:1:7
    tgspeed = i/10;
    for j = -4:1:4
        ctspeed=tgspeed+j/10;
        
        if tgspeed==0.7
            5;
        end
%         if abs(ctspeed-0.2)>1e-3 || abs(tgspeed-0.7)>1e-3
            load(['opt_result\trans_type2_' num2str(ctspeed*10) 'to' num2str(tgspeed*10) 'dms'])
        

        feature = [ctspeed; tgspeed];
        input_avg=[input_avg, feature ];
        output_alpha=[output_alpha, outputs{1}.a(1,:)'];
%         end
        if info.status~=0
            disp(['not optimal: current' num2str(ctspeed) 'and target' num2str(tgspeed)])
        end
    end
end

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
    [x_matrix y_matrix]=meshgrid(linspace(min(x(1,:)),max(x(1,:)),200),linspace(min(x(2,:)),max(x(2,:)),200));
    x_line=reshape(x_matrix,1,[]);
    y_line=reshape(y_matrix,1,[]);
    z_line=net([x_line;y_line]);
    z_matrix=[];
    for j = 1:size(z_line,1)
        z_matrix(:,:,j)=reshape(z_line(j,:),size(x_matrix,1),size(x_matrix,2));
    end
    
    for j = 1:6
        subplot(4,6,(i-1)*6+j)
        scatter3(x(1,:),x(2,:),t(j,:))
%         hold on
%         mesh(x_matrix,y_matrix,z_matrix(:,:,j));
%         hold off
        xlabel('ctspeed')
        ylabel('tgspeed')
        zlabel('\alpha')
    end
end