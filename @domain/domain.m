classdef domain
    %DOMAIN Specifies the structure of a domain in the hybrid system model
    % Description
    %   This class specifies the structure of a domain in the hybrid system
    %   model, including:
    %          * Holonomic constraints
    %          * Continuous dynamics
    %          * Guard condition
    %          * Reset map (map from the current domain to the next domain)
    %          * Controller
    %          * Zero dynamics ???
    %
    % Copyright 2014-2015 Texas A&M University AMBER Lab
    % Author: Ayonga Hereid <ayonga@tamu.edu>
    
    properties
        domainName@char % domain name
        domainIndex@double % domain index
        
    end
    
    methods
        
        %% constructor
        
        function obj = domain(name, index)
            % domain - The constructor of the class

            % Domain name
            obj.domainName  = name;
            obj.domainIndex = index; 

        end

    end
    
end

