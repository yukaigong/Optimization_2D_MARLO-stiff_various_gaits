% %% See the figure for a
% figure
% a=[];
% for i = 1:length(outputs)
%     a=outputs{i}.a(1,:);
%     subplot(2,2,i)
%     plot(1:24,a)
%     subplot(2,2,4)
%     hold on
%     plot(1:24,a)
%     hold off
% end
%% compare 1-step periodic and 3-step periodic

figure
a=[];
speed_range= -12:1:12; % dm/s
fail_speed_type3=[];
fail_speed_type1=[];

for i = speed_range
    load(['opt_result\avg_type3_' num2str(i) 'dms'])
    if info.status~=0
        fail_speed_type3 = [fail_speed_type3, i/10];
    end
    for j = 1:length(outputs)
        a=outputs{j}.a(1,:);
        subplot(5,5,i+1-speed_range(1))
        hold on
        plot(1:24,a)
        hold off
        title([num2str(i/10) ' m/s'])
    end
    load(['opt_result\avg_type1_' num2str(i) 'dms'])
    if info.status~=0
        fail_speed_type1 = [fail_speed_type1, i/10];
    end
    a=outputs{1}.a(1,:);
    subplot(5,5,i+1-speed_range(1))
    hold on
    plot(1:24,a)
    hold off
end

display(['3 step failed speed: ' num2str(fail_speed_type3) ' m/s']);
display(['1 step failed speed: ' num2str(fail_speed_type1) ' m/s']);

%% compare states and bazier curves
figure
H0 =  [0, 0, 0, .5, .5,  0,  0;...
       0, 0, 0,  0,  0, .5, .5;...
       0, 0, 0, -1,  1,  0,  0;...
       0, 0, 0,  0,  0, -1,  1];
   
y=H0*outputs{1}.q';
s=linspace(0,1,29);
a=reshape(outputs{1}.a(1,:),4,6);
for i=1:length(s)
    b(:,i)=bezier(a,s(i));
end
for i=1:4
    subplot(2,2,i)
    plot(1:29,y(i,:))
    hold on
    plot(1:29,b(i,:))
    hold off
end

%% plot 3-d curve of the gaits and see if they are smooth

outputs_name={'Stance Leg Angle', 'Swing Leg Angle', 'Stance Knee Angle', 'Swing Knee Angle'};
t=[];
speed_range= -12:1:12;
for i = speed_range
    load(['opt_result\avg_type1_' num2str(i) 'dms'])
    a=reshape(outputs{1}.a(1,:),4,6);
    s=linspace(0,1,29);
    for j=1:length(s)
        b(:,j)=bezier(a,s(j));
    end
    for j = 1:4
        subplot(2,2,j)
        hold on
        plot3(s,i/10*ones(1,length(s)),b(j,:))
        hold off
        title(outputs_name{j})
        xlabel('s')
        zlabel('deg')
        ylabel('velocity')
        grid on
    end
    if info.status ~=0 
        disp(i/12);
    end
    t=[t outputs{1}.t(1)];
end
figure
plot(speed_range,t)
