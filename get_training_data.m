gaits_type=2;
speed=0;
tgspeed=0;
ctspeed=0;

switch gaits_type
    case 1
        % one step
        for i=-12:1:12
            speed=i/10
            gaits_type=1;
            ipopt_optimization;
            save(['opt_result\avg_type1_' num2str(i) 'dms'],'outputs','x','info')
        end
        
    case 3
        % three step
        for i=-12:1:12
            speed=i/10
            gaits_type=3;
            ipopt_optimization;
            save(['opt_result\avg_type3_' num2str(i) 'dms'],'outputs','x','info')
        end
        
    case 2
        % three steps transient
        %         for i=-12:1:12
        %             ctspeed=i/10
        % %             tgspeed=ctspeed;
        %             if abs(ctspeed)<=0.4
        %             tgspeed=0;
        %             else
        %                 tgspeed=ctspeed-sign(ctspeed)*0.4;
        %             end
        %             gaits_type=2;
        %             ipopt_optimization;
        %             save(['opt_result\trans_type2_' num2str(ctspeed*10) 'to' num2str(tgspeed*10) 'dms'],'outputs','x','info')
        %         end
        for i=-4:1:4
            tgspeed=i/10
            %             tgspeed=ctspeed;
            for j = -4:1:4
                ctspeed=tgspeed+j/10;
                gaits_type=2;
                disp(['ctspeed' num2str(ctspeed) ';     tgspeed=' num2str(tgspeed)])
                ipopt_optimization;
                save(['opt_result\trans_type2_' num2str(ctspeed*10) 'to' num2str(tgspeed*10) 'dms'],'outputs','x','info')
            end
        end
        
    otherwise
        disp('This gait does not exist')
end