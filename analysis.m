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
%% compare 3-step periodic and 3-step transient ( the transient is transient from and to same gaits
figure
a=[];
speed_range= -12:1:12; % dm/s
fail_speed_type3=[];
fail_speed_type2=[];

for i = speed_range
    load(['opt_result\avg_type3_' num2str(i) 'dms'])
    if info.status~=0
        fail_speed_type3 = [fail_speed_type3, i/10];
    end
    for j = 1:length(outputs)
        a=outputs{j}.a(1,:);
        a=reshape(outputs{j}.a(1,:),4,6);
        s=linspace(0,1,29);
        for j=1:length(s)
            b(:,j)=bezier(a,s(j));
        end
        for k=1:4
            figure(k)
            subplot(5,5,i+1-speed_range(1))
            hold on
            plot(s,b(k,:))
            hold off
            title([num2str(i/10) ' m/s'])
        end
    end
    
    load(['opt_result\trans_type2_' num2str(i) 'to' num2str(i) 'dms'])
    if info.status~=0
        fail_speed_type2 = [fail_speed_type2, i/10];
    end
    for j = 1:length(outputs)
        a=outputs{j}.a(1,:);
        a=reshape(outputs{j}.a(j,:),4,6);
        s=linspace(0,1,29);
        for j=1:length(s)
            b(:,j)=bezier(a,s(j));
        end
        for k=1:4
            figure(k)
            subplot(5,5,i+1-speed_range(1))
            hold on
            plot(s,b(k,:))
            hold off
            title([num2str(i/10) ' m/s'])
        end
    end
end
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
        a=reshape(outputs{j}.a(1,:),4,6);
        s=linspace(0,1,29);
        for j=1:length(s)
            b(:,j)=bezier(a,s(j));
        end
        for k=1:4
            figure(k)
            subplot(5,5,i+1-speed_range(1))
            hold on
            plot(s,b(k,:))
            hold off
            title([num2str(i/10) ' m/s'])
        end
    end
    load(['opt_result\avg_type1_' num2str(i) 'dms'])
    if info.status~=0
        fail_speed_type1 = [fail_speed_type1, i/10];
    end
    a=outputs{1}.a(1,:);
        a=reshape(outputs{1}.a(1,:),4,6);
        s=linspace(0,1,29);
        for j=1:length(s)
            b(:,j)=bezier(a,s(j));
        end
        for k=1:4
            figure(k)
            subplot(5,5,i+1-speed_range(1))
            hold on
            plot(s,b(k,:))
            hold off
            title([num2str(i/10) ' m/s'])
        end
%         subplot(5,5,i+1-speed_range(1))
%     hold on
%     plot(1:24,a)
%     hold off
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
%% examine the result of 3-step transient
fail_speed_type2 =[];
exceed=[];
speed_range= -12:1:-5; % dm/s
for i = speed_range
    ctspeed=i/10;
    if abs(ctspeed)<=0.4
        tgspeed=0;
    else
        tgspeed=ctspeed-sign(ctspeed)*0.4;
    end
    load(['opt_result\trans_type2_' num2str(ctspeed*10) 'to' num2str(tgspeed*10) 'dms'])
    if info.status~=0
        fail_speed_type2 = [fail_speed_type2, i/10];
    end
    if info.status==-1
        exceed=[exceed, i/10];
    end
end
display(['3 step transient failed speed: ' num2str(fail_speed_type2) ' m/s']);
display(['3 step transient exceed speed: ' num2str(exceed) ' m/s']);
%% examine the ground force
figure
speed_range= -12:1:12; % dm/s
for i = speed_range
    ctspeed=i/10
    if abs(ctspeed)<=0.4
        tgspeed=0;
    else
        tgspeed=ctspeed-sign(ctspeed)*0.4;
    end
%     load(['opt_result\trans_type2_' num2str(ctspeed*10) 'to' num2str(tgspeed*10) 'dms'])
%     load(['opt_result\avg_type3_' num2str(i) 'dms'])
    load(['opt_result\avg_type1_' num2str(i) 'dms'])
    F1=[];
    F2=[];
    for j = 1:length(outputs)
        F1=[F1;outputs{j}.Fe(:,1)];
        F2=[F2;outputs{j}.Fe(:,2)];
    end
    subplot(5,5,i+13)
%     plot(1:length(F1),F1)
%     hold on
    plot(1:length(F2),F2)
        
end
