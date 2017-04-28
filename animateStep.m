function animateStep(input)
    % Draws walker in right stance (for initialization testin)
    figure
    nNodes = size(input.q,1);
    offset = [0;0];
    for steps=1:10
        for i=1:1:nNodes
            q = input.q(i,:)';
            q(1:2) = q(1:2) + offset;
            [pT, pH, p1R, p2R, p3R, p4R, p1L, p2L, p3L, p4L, pcm, pcmT, pcm1, pcm2, pcm3, pcm4, pcm1L, pcm2L, pcm3L, pcm4L] =  PointsAtrias2D(q);
    %         pStanceFoot = p4R;
    %         [pT,pHip,p1R,p2R,p3R,p4R,p1L,p2L,p3L,p4L,pcm,pcmT,pcm1R,pcm2R,pcm3R,pcm4R,pcm1L,pcm2L,pcm3L,pcm4L] = PointsAtrias2DWorldFrame(q,pStanceFoot);

            hold off
            plot ([-10 10],[0,0],'g','LineWidth',3); % ground
            hold on

            % Draw right leg
            plot([p4R(1) p2R(1)],[p4R(2) p2R(2)],'r','LineWidth',2);
            plot([p2R(1) pH(1)],[p2R(2) pH(2)],'r','LineWidth',2);

            plot([p3R(1) p1R(1)],[p3R(2) p1R(2)],'r','LineWidth',2);
            plot([p1R(1) pH(1)],[p1R(2) pH(2)],'r','LineWidth',2);

            % Draw left leg
            plot([p4L(1) p2L(1)],[p4L(2) p2L(2)],'g','LineWidth',2);
            plot([p2L(1) pH(1)],[p2L(2) pH(2)],'g','LineWidth',2);

            plot([p3L(1) p1L(1)],[p3L(2) p1L(2)],'g','LineWidth',2);
            plot([p1L(1) pH(1)],[p1L(2) pH(2)],'g','LineWidth',2);

            plot([pH(1) pT(1)],[pH(2) pT(2)],'k','LineWidth',2);
            axis([-1+pH(1) 3+pH(1) -.5 2])
            %%%%%%get video%%%%%
            F((steps-1)*nNodes+i) = getframe(gcf);
            %%%%%%%%%
            pause(input.t(1)/nNodes)

            if i == nNodes
                offset = p4L;
            end
        end
    end
    %%%%%make video
    vi = VideoWriter('video', 'MPEG-4');
    set(vi, 'FrameRate', nNodes/input.t(1));
    open(vi);
    writeVideo(vi, F);
    close(vi);
    %%%%%%%%%%%%%%
end

