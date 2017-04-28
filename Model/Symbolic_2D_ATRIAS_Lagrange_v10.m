% Modified May 15th, 2015 (Cleaned up code)
% Symbolic_2D_ATRIAS_Lagrange_v10.m
%
% Reflects correction of error in torso mass

clear all
disp('Cleared Workspace')

syms g L1 L2 L3 L4 m1 m2 m3 m4
syms Jcm1 Jcm2 Jcm3 Jcm4
syms ellycm1 ellycm2 ellycm3 ellycm4 ellzcm1 ellzcm2 ellzcm3 ellzcm4
syms LT mT JcmT mH JcmH ellycmT  ellzcmT
syms K1 K2 Kd1 Kd2  %Spring constants and damping constants
syms Jrotor1 Jrotor2 Jgear1 Jgear2 R1  R2

syms qT yH zH dqT dyH dzH
syms q1R q2R dq1R dq2R
syms q1L q2L dq1L dq2L
syms qm1R qm2R dqm1R dqm2
syms qm1L qm2L dqm1L dqm2L

syms qgr1R qgr2R dqgr1R dqgr2R     %gear reducer coordinates; are probably better numerically
syms qgr1L qgr2L dqgr1L dqgr2L    %than the motor coordinates, due to the low inertia of the rotors
syms yH zH 


%springs:  0 = springs not included in stance leg of model, 1 = springs included in stance leg
%
%springsLeft:  0 = springs not included in swing leg of model, 1 = springs included in swing leg
%
%HipPositionChoice:
% 0 = fix pHip=[yH;zH]=[0;0]; useful for checking model in a
%     simplify situation
% 1 = fix stance leg end at [0;0] and hips computed
%     from leg angles; normal SS model
% 2 = Attach extra coordinates [yLeg;zLeg] at end of stance leg so
%     that p4=[yLeg;zLeg] and hips are computed from leg angles
%     This gives the normal impact model
% 3 = Attach extra coordiantes at  [yH;zH] at the hip,
%     so pHip =[yH;zH]  Unsure where to use it, but
%     it seems like a nice idea.


addpath('ModelParameters')
addpath('GlobalVariables')

% Execute FilePaths script to get all important file paths
% This must be executed from the GlobalVariables folder
cd('GlobalVariables')
FilePaths
cd(MODEL_PATH)

[g, mTotal, L1, L2, L3, L4, m1, m2, m3, m4, Jcm1, Jcm2, Jcm3, Jcm4,  ...
 ellzcm1, ellzcm2, ellzcm3, ellzcm4, ellycm1, ellycm2, ellycm3, ellycm4, ...
 LT, mT, JcmT, ellzcmT, ellycmT, mH, JcmH, K1, K2, Kd1, Kd2, Jrotor1, Jrotor2, Jgear1, ...
 Jgear2, R1,  R2] = modelParametersAtrias_v05;

%% Select Model

Model='flight';
switch lower(Model)     %add cases as needed

    case {'dynamic'}  % Standard SS Model with stance on RIGHT Leg
        springs=1;
        springsLeft=1;
        HipPositionChoice=1;
        disp('Building SS Model')

    case {'dynamicreduced'}  % Reduced SS Model...no springs
        springs=0;
        springsLeft=0;
        HipPositionChoice=3; % Might have to modify hip position for cases
        disp('Building SS Model with No Springs')

    case {'impact'}  % Standard DS (impact) model
        springs=1;
        springsLeft=1;
        HipPositionChoice=1;
        disp('Building DS Impact Model')

    case {'impactreduced'}  % No springs in the impact model
        springs=0;
        springsLeft=0;
        HipPositionChoice=2;
        disp('Building DS Impact Model with No Springs')

    case {'flight'}  % Unconstrained or Flight Phase Model
        springs=0;
        springsLeft=0;
        HipPositionChoice=3;
        disp('Building Flight Phase Model')

    case {'dynamic_leftstance'}  % SS Model with Stance on LEFT Leg
        springs=1;
        springsLeft=1;
        HipPositionChoice=4;
        disp('Building SS Model')


    otherwise
        disp('Unknown Model Choice')
        return
end

%% Set up coordinates

if and(springs,springsLeft)
    q=[qT;q1R;q2R;qgr1R;qgr2R;q1L;q2L;qgr1L;qgr2L];
    dq=[dqT;dq1R;dq2R;dqgr1R;dqgr2R;dq1L;dq2L;dqgr1L;dqgr2L];
elseif springs
    q=[qT;q1R;q2R;qgr1R;qgr2R;q1L;q2L];
    dq=[dqT;dq1R;dq2R;dqgr1R;dqgr2R;dq1L;dq2L];
else
    q=[qT;q1R;q2R;q1L;q2L]; % this is the selected case, q and dq should be in R5.
    dq=[dqT;dq1R;dq2R;dq1L;dq2L];
end


%% Get Absolute Angles

R = @(x) [cos(x), -sin(x); sin(x), cos(x)];

% Right Leg
th1R = qT + q1R;  %absolute orientation of link 1
th2R = qT + q2R;  %absolute orientation of link 2
dth1R = dqT + dq1R;
dth2R = dqT + dq2R;

% Left Leg
th1L = qT + q1L;  %absolute orientation of link 1
th2L = qT + q2L;  %absolute orientation of link 2
dth1L = dqT + dq1L;
dth2L = dqT + dq2L;

%% Get Positions

% Set the Hip position
pHip = [yH; zH];
if HipPositionChoice == 1 % leg end is fixed % This should be how postitions are set up
    p1R = pHip + R(th1R)*[0;L1];
    p2R = pHip +  R(th2R)*[0;L2];
    p3R = p1R + R(th2R)*[0;L3];
    p4R = p2R + R(th1R)*[0;L4];
    pT = pHip + R(qT)*[0;LT];
    S=solve(p4R(1),p4R(2),yH,zH);
    yH=S.yH; zH=S.zH;
    pHip=[yH;zH];
elseif HipPositionChoice == 0  %Hip position is fixed
    yH = 0*yH; zH = 0*zH;
    pHip = [yH;zH];
elseif HipPositionChoice == 3 % Hip position is free with coordinates [yH; zH]
    q=[yH;zH;q];
    dq=[dyH;dzH;dq];
elseif HipPositionChoice == 2 % leg end is [yLeg,zLeg]
    p1R = pHip + R(th1R)*[0;L1];
    p2R = pHip +  R(th2R)*[0;L2];
    p3R = p1R + R(th2R)*[0;L3];
    p4R = p2R + R(th1R)*[0;L4];
    pT = pHip + R(qT)*[0;LT];
    S=solve(p4R(1),p4R(2),yH,zH);
    yH=S.yH; zH=S.zH;
    syms yLeg zLeg dyLeg dzLeg
    q=[yLeg;zLeg;q];
    dq=[dyLeg;dzLeg;dq];
    pHip=[yH;zH] + [yLeg;zLeg];
elseif HipPositionChoice == 4 % left leg end is fixed
    p1L = pHip + R(th1L)*[0;L1];
    p2L = pHip + R(th2L)*[0;L2];
    p3L = p1L + R(th2L)*[0;L3];
    p4L = p2L + R(th1L)*[0;L4];
    pT = pHip + R(qT)*[0;LT];
    S=solve(p4L(1),p4L(2),yH,zH);
    yH=S.yH; zH=S.zH;
    pHip=[yH;zH];
end

% position vectors relative to hip position
p1R = pHip + R(th1R)*[0;L1]; p1R = simplify(p1R);
p2R = pHip +  R(th2R)*[0;L2]; p2R = simplify(p2R);
p3R = p1R + R(th2R)*[0;L3]; p3R = simplify(p3R);
p4R = p2R + R(th1R)*[0;L4]; p4R = simplify(p4R);
pT = pHip + R(qT)*[0;LT]; pT=simplify(pT);

disp('Stance leg end = p4R = [0,0] means standard SS model')
disp(p4R)
disp('Hip position')
disp(pHip)

% Get Center of Mass positions
pcm1R = pHip + R(th1R)*[ellycm1;ellzcm1];
pcm2R = pHip +  R(th2R)*[ellycm2;ellzcm2];
pcm3R = p1R + R(th2R)*[ellycm3;ellzcm3];
pcm4R = p2R + R(th1R)*[ellycm4;ellzcm4];

pcmT = pHip + R(qT)*[ellycmT;ellzcmT];

p1L = pHip + R(th1L)*[0;L1]; p1L = simplify(p1L);
p2L = pHip +  R(th2L)*[0;L2]; p2L = simplify(p2L);
p3L = p1L + R(th2L)*[0;L3]; p3L = simplify(p3L);
p4L = p2L + R(th1L)*[0;L4]; p4L = simplify(p4L);

pcm1L = pHip + R(th1L)*[ellycm1;ellzcm1];
pcm2L = pHip +  R(th2L)*[ellycm2;ellzcm2];
pcm3L = p1L + R(th2L)*[ellycm3;ellzcm3];
pcm4L = p2L + R(th1L)*[ellycm4;ellzcm4];

Mtotal = mT + mH + 2*(m1 + m2 + m3 + m4);
pcm = (mT*pcmT + mH*pHip + m1*pcm1R + m2*pcm2R + m3*pcm3R + m4*pcm4R +  m1*pcm1L + m2*pcm2L + m3*pcm3L + m4*pcm4L)/Mtotal;
pcm=simplify(pcm);

% Calculate Velocities
vHip=jacobian(pHip,q)*dq;
vcm1R=jacobian(pcm1R,q)*dq;
vcm2R=jacobian(pcm2R,q)*dq;
vcm3R=jacobian(pcm3R,q)*dq;
vcm4R=jacobian(pcm4R,q)*dq;

vcmT=jacobian(pcmT,q)*dq;

vcm1L=jacobian(pcm1L,q)*dq;
vcm2L=jacobian(pcm2L,q)*dq;
vcm3L=jacobian(pcm3L,q)*dq;
vcm4L=jacobian(pcm4L,q)*dq;

disp('Key positions and velocities computed')

%% Kinetic energy

disp('Calculating Kinetic Energy')
KET=(1/2)*mT*vcmT.'*vcmT + (1/2)*JcmT*(dqT)^2;
KET=simplify(KET);

KEH=(1/2)*mH*vHip.'*vHip + (1/2)*JcmH*(dqT)^2;
KEH=simplify(KEH);

KE1R=(1/2)*m1*vcm1R.'*vcm1R + (1/2)*Jcm1*(dth1R)^2;
KE1R=simplify(KE1R);

KE2R=(1/2)*m2*vcm2R.'*vcm2R + (1/2)*Jcm2*(dth2R)^2;
KE2R=simplify(KE2R);

KE3R=(1/2)*m3*vcm3R.'*vcm3R + (1/2)*Jcm3*(dth2R)^2;
KE3R=simplify(KE3R);

KE4R=(1/2)*m4*vcm4R.'*vcm4R + (1/2)*Jcm4*(dth1R)^2;
KE4R=simplify(KE4R);

KE1L=(1/2)*m1*vcm1L.'*vcm1L + (1/2)*Jcm1*(dth1L)^2;
KE1L=simplify(KE1L);

KE2L=(1/2)*m2*vcm2L.'*vcm2L + (1/2)*Jcm2*(dth2L)^2;
KE2L=simplify(KE2L);

KE3L=(1/2)*m3*vcm3L.'*vcm3L + (1/2)*Jcm3*(dth2L)^2;
KE3L=simplify(KE3L);

KE4L=(1/2)*m4*vcm4L.'*vcm4L + (1/2)*Jcm4*(dth1L)^2;
KE4L=simplify(KE4L);

% Motor coordinates
if springs
%
else % gear variables are same as link variables
    qgr1R=q1R;
    qgr2R=q2R;
    dqgr1R=dq1R;
    dqgr2R=dq2R;
end

if springsLeft
%
else
    qgr1L=q1L;
    qgr2L=q2L;
    dqgr1L=dq1L;
    dqgr2L=dq2L;

end

% Calcualte motor position using gear ratios
qm1R=qgr1R*R1; % motor position related to _ by gear ratio
qm2R=qgr2R*R2;
qm1L=qgr1L*R1;
qm2L=qgr2L*R2;

dqm1R=dqgr1R*R1;
dqm2R=dqgr2R*R2;
dqm1L=dqgr1L*R1;
dqm2L=dqgr2L*R2;

% absolute motor angular position
thm1R=qT+qm1R;
thm2R=qT+qm2R;
dthm1R=dqT+dqm1R;
dthm2R=dqT+dqm2R;

thm1L=qT+qm1L;
thm2L=qT+qm2L;
dthm1L=dqT+dqm1L;
dthm2L=dqT+dqm2L;

% absolute gear reducer angular velocity
dthr1R=dqT+dqgr1R; 
dthr2R=dqT+dqgr2R;
dthr1L=dqT+dqgr1L;
dthr2L=dqT+dqgr2L;

% Calculate Motor Kinetic Energy
KEm1R = (1/2)*Jrotor1*(dthm1R)^2 + (1/2)*Jgear1*(dthr1R)^2;
KEm2R = (1/2)*Jrotor2*(dthm2R)^2 + (1/2)*Jgear2*(dthr2R)^2; % Other leg
KE_motorsRight = KEm1R + KEm2R;
KE_motorsRight=simplify(KE_motorsRight);

KEm1L = (1/2)*Jrotor1*(dthm1L)^2 + (1/2)*Jgear1*(dthr1L)^2;
KEm2L = (1/2)*Jrotor2*(dthm2L)^2 + (1/2)*Jgear2*(dthr2L)^2;
KE_motorsLeft = KEm1L + KEm2L;
KE_motorsLeft=simplify(KE_motorsLeft);

disp('Kinetic Energy of Motors and Gear Reducers computed')

% Calculate Total Kinetic Energy
KE_LegRight = KE1R + KE2R + KE3R + KE4R;
KE_LegRight = simplify(KE_LegRight);

KE_LegLeft = KE1L + KE2L + KE3L + KE4L;
KE_LegLeft = simplify(KE_LegLeft);

KE = KET + KEH + KE_motorsRight + KE_LegRight + KE_motorsLeft + KE_LegLeft;

disp('Total Kinetic Energy computed')



%% Potential energy

PE = g*Mtotal*pcm(2);

Phi_springs=[]; Kdamping=[];

if springs
    % spring potential energy
    SE = (1/2)*K1*( q1R-qgr1R )^2 + (1/2)*K2*(q2R - qgr2R)^2;
    PE = PE + SE;
    Phi_springs=[Phi_springs; q1R-qgr1R; q2R-qgr2R];
    Kdamping=[Kdamping,Kd1,Kd2];
end

if springsLeft
    % spring potential energy
    SE_Left = (1/2)*K1*( q1L-qgr1L )^2 + (1/2)*K2*(q2L - qgr2L)^2;
    PE = PE + SE_Left;
    Phi_springs=[Phi_springs; q1L-qgr1L; q2L - qgr2L];
    Kdamping=[Kdamping,Kd1,Kd2];
end

disp('PE computed')



%% Compute D 
D=jacobian(KE,dq);
D=jacobian(D,dq);

for i=1:length(q)
    for j=i:length(q)
        D(i,j)=simplify(D(i,j));
        D(j,i)=D(i,j);
        disp(['D(',num2str(i),',',num2str(j),')= ',char(D(i,j))])
    end
end

%% Compute C

C=0*D;
for k=1:length(q)
    for j=1:length(q)
        for i=1:length(q)
            C(k,j)=C(k,j)+(1/2)*(diff(D(k,j),q(i))+diff(D(k,i),q(j))-diff(D(i,j),q(k)))*dq(i);
        end
    end
end

for i=1:length(q)
    for j=1:length(q)
        C(i,j)=simplify(C(i,j));
        disp(['C(',num2str(i),',',num2str(j),')= ',char(C(i,j))])
    end
end


%% Compute G
G=jacobian(PE,q).';
G=simplify(G);

for i=1:length(q)
    disp(['G(',num2str(i),')= ',char(G(i))])
end

%% Compute B

% Actuation
Phi=[qm1R;qm2R;qm1L;qm2L];
B=jacobian(Phi,q);
B=B.';

%%% Harmonic Drive Friction
%% Actuation
Phi_HarmonicDrive=[qgr1R;qgr2R;qgr1L;qgr2L];
B_HarmonicDrive=jacobian(Phi_HarmonicDrive,q);
B_HarmonicDrive=B_HarmonicDrive.';


%% Spring Damping
if ~isempty(Phi_springs) % Will not be the case because Phi_springs was not set in potential energy above.
    JacPhi_springs = jacobian(Phi_springs,q);
    B_springs = JacPhi_springs.';
    Kdamping = diag(Kdamping);
    DampingSpringTorque = B_springs*Kdamping*JacPhi_springs*dq;
    DampingSpringTorque = simplify(DampingSpringTorque);
else
    DampingSpringTorque=zeros(length(q),1)*Kd1;
end


%% Used in the impact model and flight model

JacobianLeftFoot=jacobian(p4L,q);
EL=JacobianLeftFoot.';
E=JacobianLeftFoot;

JacobianRightFoot=jacobian(p4R,q);
ER=JacobianRightFoot.';


%% Write Files
switch lower(Model)

    case {'dynamic','dynamicreduced'}

        % Create Lagrange Model Function
        matlabFunction(D,C,G,B,B_HarmonicDrive, 'file', [AUTOGEN_PATH ,'\LagrangeModelAtrias2D'], 'vars', {q,dq});
      
        % Create Function to retrieve points
        StringPoints = ['pT   '; 'pHip '; 'p1R  '; 'p2R  '; 'p3R  '; 'p4R  '; 'p1L  '; 'p2L  '; 'p3L  '; 'p4L  '; 'pcm  '; 'pcmT '; 'pcm1R'; 'pcm2R'; 'pcm3R'; 'pcm4R'; 'pcm1L'; 'pcm2L'; 'pcm3L'; 'pcm4L'];
        PointsCellArrayString = cellstr(StringPoints);
        matlabFunction(pT, pHip, p1R, p2R, p3R, p4R, p1L, p2L, p3L, p4L, pcm, pcmT, pcm1R, pcm2R, pcm3R, pcm4R, pcm1L, pcm2L, pcm3L, pcm4L, 'file', [AUTOGEN_PATH ,'\PointsAtrias2D'], 'vars', {q}, 'outputs', PointsCellArrayString);

        
        % Create Function to retreive world points
        syms pS_h pS_v
        pStance = [pS_h; pS_v];
        matlabFunction(pT+pStance, pHip+pStance, p1R+pStance, p2R+pStance, p3R+pStance, p4R+pStance, p1L+pStance, p2L+pStance, p3L+pStance, p4L+pStance, pcm+pStance, pcmT+pStance, pcm1R+pStance, pcm2R+pStance, pcm3R+pStance, pcm4R+pStance, pcm1L+pStance, pcm2L+pStance, pcm3L+pStance, pcm4L+pStance, 'file', [AUTOGEN_PATH ,'\PointsAtrias2DWorldFrame'], 'vars', {q, pStance}, 'outputs', PointsCellArrayString);
        
        
        % Create Function to retrieve velocities and accelerations
        vcm=jacobian(pcm,q)*dq;
        J_cm=jacobian(pcm,q);
        dJ_cm=jacobian(J_cm*dq,q);
        vp4L = jacobian(p4L,q)*dq;
        matlabFunction(vHip, vcm, J_cm, dJ_cm, vp4L, 'file', [AUTOGEN_PATH ,'\VelAccelAtrias2D'], 'vars', {q,dq});
        
        % Create Function to retrieve energies
        StringEnergies = ['PE   '; 'KE   '; 'KET  '; 'KE1R '; 'KE2R '; 'KE3R '; 'KE4R '; 'KEm1R'; 'KEm2R'; 'KE1L '; 'KE2L '; 'KE3L '; 'KE4L '; 'KEm1L'; 'KEm2L'];
        EnergiesCellArrayString = cellstr(StringEnergies);
        matlabFunction(PE, KE, KET, KE1R, KE2R, KE3R, KE4R, KEm1R, KEm2R, KE1L, KE2L, KE3L, KE4L, KEm1L, KEm2L, 'file', [AUTOGEN_PATH ,'\EnergiesAtrias2D'], 'vars', {q,dq}, 'outputs', EnergiesCellArrayString);
     
        % Save Workspace
        Work_name = 'Mat\Work_Symbolic_2D_ATRIAS_Lagrange_JWG_';
        Work_name2=[Work_name,Model,'_',datestr(now,30)];
        eval(['save  ',Work_name2])
        
    case {'impact'}
        
        matlabFunction(D, E, 'file', [AUTOGEN_PATH ,'\ImpactModelAtrias2D'], 'vars', {q,dq});
        
        Work_name = 'Mat\Work_Symbolic_2D_ATRIAS_Lagrange_JWG_';
        Work_name2=[Work_name,Model,'_',datestr(now,30)];
        eval(['save  ',Work_name2])

    case {'impactreduced'}  % No springs in the impact model
        
        matlabFunction(D, E, 'file', [AUTOGEN_PATH ,'\ImpactModelAtrias2D_NoSprings'], 'vars', {q});
        
        Work_name = 'Mat\Work_Symbolic_2D_ATRIAS_Lagrange_JWG_';
        Work_name2=[Work_name,Model,'_',datestr(now,30)];
        eval(['save  ',Work_name2])

    case {'flight'}

        Work_name = 'Mat\Work_Symbolic_2D_ATRIAS_Lagrange_JWG_';
        Work_name2=[Work_name,Model,datestr(now,30)];
        eval(['save  ',Work_name2])

        fcn_name = 'LagrangeModelAtriasFlight2D';
%         generate_model_flight

        PointsVector=[pT pHip p1R p2R p3R p4R p1L p2L p3L p4L pcm pcmT pcm1R pcm2R pcm3R pcm4R pcm1L pcm2L pcm3L pcm4L];
        StringPointsVector='pT pHip p1 p2 p3 p4 p1L p2L p3L p4L pcm pcmT pcm1 pcm2 pcm3 pcm4 pcm1L pcm2L pcm3L pcm4L';

        fcn_name = 'PointsAtrias2D';
%         generate_points

        %         fcn_name = 'PointsAtrias2DWorldFrame';
        %         generate_points_WorldFrame
        %
        vcm=jacobian(pcm,q)*dq;
        J_cm=jacobian(pcm,q);
        dJ_cm=jacobian(J_cm*dq,q);

        VelAccelVector=[vcm J_cm dJ_cm];
        StringVelAccelVector='vcm J_cm dJ_cm';

        %         PrimaryVelocities=[vHip vcm];
        %         StringPrimaryVelocities='vHip vcm';

        AccelJacobians=[J_cm dJ_cm];
        StringAccelJacobians='J_cm dJ_cm';

        fcn_name = 'VelAccelAtrias2D';
%         generate_velocties_accelerations
        %
        %
        %         Energies=[PE KE KET KE1 KE2 KE3 KE4 KEm1 KEm2 KE1L KE2L KE3L KE4L KEm1L KEm2L];
        %         StringEnergies='PE KE KET KE1 KE2 KE3 KE4 KEm1 KEm2 KE1L KE2L KE3L KE4L KEm1L KEm2L';
        %

        %         fcn_name = 'EnergiesAtrias2D';
        %         generate_energies
        %
        Work_name = 'Mat\Work_Symbolic_2D_ATRIAS_Lagrange_JWG_';
        %Work_name2=[Work_name,datestr(now,7),datestr(now,3),datestr(now,10)];
        Work_name2=[Work_name,Model,datestr(now,30)];
        eval(['save  ',Work_name2])

    case {'dynamic_leftstance'}

        Work_name = 'Mat\Work_Symbolic_2D_ATRIAS_Lagrange_JWG_';
        %Work_name2=[Work_name,datestr(now,7),datestr(now,3),datestr(now,10)];
        Work_name2=[Work_name,Model,datestr(now,30)];
        eval(['save  ',Work_name2])

        fcn_name = 'LagrangeModelAtrias2D_Left';
        generate_model

    otherwise
        disp('ERROR ON MODEL CHOICE')
end

