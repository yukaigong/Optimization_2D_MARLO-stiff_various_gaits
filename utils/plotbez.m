function plotbez(alpha,s,Bbar,varargin)
    
    if nargin<2 || isempty(s)
        s=0:0.001:1;
    end
    B(size(alpha,1),length(s)) = 0;
    
    if nargin>2 && ~isempty(Bbar) && isequal(size(alpha), size(Bbar))
        alphabar = Bbar;
        Bbar = B;
        for k=1:length(s)
            Bbar(:,k) = bezier(alphabar,s(k));
        end
    end
    
    for k=1:length(s)
        B(:,k) = bezier(alpha,s(k));
    end
    if nargin<3 || isempty(Bbar)
        plot(s,B.',varargin{:})
    else
        plot(s,B.'); hold on;
        if ~verLessThan('MATLAB', '8.4.0')
            set(gca, 'ColorOrderIndex', 1);
        end
        plot(s,Bbar,'--'); hold off
    end
end
