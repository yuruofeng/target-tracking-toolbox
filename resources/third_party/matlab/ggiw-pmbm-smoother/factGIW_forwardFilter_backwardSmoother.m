classdef factGIW_forwardFilter_backwardSmoother<handle
    
    properties
        
        models;
        
        m0;
        P0;
        v0;
        V0;
        
        mpred;
        Ppred;
        vpred;
        Vpred;
        
        mup;
        Pup;
        vup;
        Vup;
        
        msm;
        Psm;
        vsm;
        Vsm;
        
        GWD;
        KIN;
        EXT;
        
    end
    
    methods
        
        function forwardFilter(obj,Z)
            
            % Number of time steps
            Nt = numel(Z);
            
            %%%%%%%%%%%%%%%%%%%%%%
            % Forward filter
            %%%%%%%%%%%%%%%%%%%%%%
            [obj.mpred,obj.Ppred,obj.vpred,obj.Vpred,obj.mup,obj.Pup,obj.vup,obj.Vup] = ...
                factorizedGIWfilter(obj.m0,obj.P0,obj.v0,obj.V0,Nt,Z,obj.models);
            
        end
        
        function backwardSmoother(obj)
            
            %%%%%%%%%%%%%%%%%%%%%%
            % Backwards smoother
            %%%%%%%%%%%%%%%%%%%%%%
            [obj.msm,obj.Psm,obj.vsm,obj.Vsm] = ...
                factorizedGIWsmoother(obj.mpred,obj.Ppred,obj.vpred,obj.Vpred,...
                obj.mup,obj.Pup,obj.vup,obj.Vup,obj.models);
            
        end
        
        function GWDmetric(obj,xtrue,Xtrue)
            
            %%%%%%%%%%%%%%%%%%%%%%
            % GWD metric
            %%%%%%%%%%%%%%%%%%%%%%
            [obj.GWD, obj.KIN, obj.EXT] = ...
                computeGWDmetric(xtrue,Xtrue,obj.mpred,obj.vpred,obj.Vpred,...
                obj.mup,obj.vup,obj.Vup,obj.msm,obj.vsm,obj.Vsm);
    
        end
        
    end
    
end