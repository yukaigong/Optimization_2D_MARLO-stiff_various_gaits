input_avg=[];
output_alpha=[];

%% retrieve data from mat files
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
        for i = -11:1:11
            ctspeed = i/10;
            if abs(ctspeed)<=0.4
                tgspeed=0;
            else
                tgspeed=ctspeed-sign(ctspeed)*0.4;
            end
            load(['opt_result\trans_type2_' num2str(ctspeed*10) 'to' num2str(tgspeed*10) 'dms'])
            theta_init = 1/2*(outputs{1}.q(1,4)+outputs{1}.q(1,5));
            dtheta_init = 1/2*(outputs{1}.dq(1,4)+outputs{1}.dq(1,5));
            tgspeed = 0;
            feature = [theta_init; dtheta_init];
            input_avg=[input_avg, feature ];
            output_alpha=[output_alpha, outputs{1}.a(1,:)'];
            
%             for j = 1:length(outputs)
%                 theta_init = 1/2*(outputs{j}.q(1,4)+outputs{j}.q(1,5));
%                 dtheta_init = 1/2*(outputs{j}.dq(1,4)+outputs{j}.dq(1,5));
%                 tgspeed = 0;
%                 feature = [theta_init; dtheta_init];
%                 input_avg=[input_avg, feature ];
%                 output_alpha=[output_alpha, outputs{j}.a(1,:)'];
%             end
        end
end

%% re-arrange the bezier parameters in 6;6;6;6
x=input_avg;
re_output_alpha=zeros(size(output_alpha));
for i = 1:4
    index_1=(i-1)+[1 5 9 13 17 21];
    index_2=(i-1)*6+[1 2 3 4 5 6];
    re_output_alpha(index_2,:)=output_alpha(index_1,:);
end

%% do the training and plot the result

switch gaits_type
    case 1
        for i = 1:4
            t = re_output_alpha(1+6*(i-1):6+6*(i-1),:);
            nntraining_periodic;
            x_line=linspace(min(x),max(x),200);
            y_line=net(x_line);
            for j = 1:6
                subplot(4,6,(i-1)*6+j)
                scatter(x,t(j,:))
                hold on;
                plot(x_line,y_line(j,:),'LineWidth',1);
                hold off;
            end
        end
    case 2
         for i = 1:4
            t = re_output_alpha(1+6*(i-1):6+6*(i-1),:);
            nntraining_periodic;
            [x_matrix y_matrix]=meshgrid(linspace(min(x(1,:)),max(x(1,:)),200),linspace(min(x(2,:)),max(x(2,:)),200));
%             for j = 1:size(x_line,1)*size(x_line,2)
%                 xy_line(:,j)=[x_matrix(j); y_matrix(j)];
%             end
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
                xlabel('\theta_{init}')
                ylabel('d\theta_{init}')
                zlabel('\alpha')
%                 hold on;
%                 mesh(x_matrix,y_matrix,z_matrix(:,:,j));
%                 hold off;
            end
         end

end




    