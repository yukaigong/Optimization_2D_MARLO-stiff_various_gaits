
global Swap dim_q dim_qe Gineq Geq

%% Dimensions
dim_qe= 7; % Springs in impact model % Two more states for foot position?
dim_q = 5; % All springs included in SS model


%% Swap Legs: Swap matrix
Id=eye(dim_q);
if dim_q == 9
    Index=[1,6:9,2:5];
    Swap=Id(Index,:);
elseif dim_q==5 % This will be the swap used.
    Index=[1,4,5,2,3];
    Swap=Id(Index,:);
elseif dim_q == 7
    Index=[1,6,7,6,7,2,3]; % Need to think about this!
    Swap=Id(Index,:);
end


