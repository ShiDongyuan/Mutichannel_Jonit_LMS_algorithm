classdef Fx_P1_LMS
    properties
         Wc 
         Xd 
         Sec
         Fd
         len
    end
    methods
        function obj = Fx_P1_LMS(N, Sec)
            obj.Wc = zeros(1,N);
            obj.Xd = zeros(N,1);
            obj.Fd = zeros(N,1);
            obj.Sec= Sec       ;
            obj.len= length(Sec);
        end
        function [Er,obj]= controller(obj,muw,xin,dis)
            N  = length(xin)     ;
            Yd = zeros(obj.len,1); 
			Wd = zeros(obj.len,length(obj.Wc));
            Er = zeros(N,1)      ; 
			XM_d = zeros(obj.len, obj.len);
            Xw_d = zeros(length(obj.Wc),obj.len);
            for i = 1:N
                obj.Xd = [xin(i); obj.Xd(1:end-1)]  ;
				XM_d   = [obj.Xd(1:obj.len),XM_d(:,1:end-1)];
                Xw_d   = [obj.Xd,Xw_d(:,1:end-1)];
                y      =  obj.Wc * obj.Xd           ;
                Yd     = [y;Yd(1:end-1)]            ;
                e      = dis(i) - obj.Sec * Yd      ;
                fx     = obj.Sec * obj.Xd(1:obj.len);
                obj.Fd = [fx;obj.Fd(1:end-1)]       ;
                    Pow    = (obj.Sec.^2) * (vecnorm(XM_d).^2)' ;
                    if Pow == 0 
                        Pow = 1e-12 ;
                    end
                    %obj.Wc = (1/obj.len)*(sum(Wd,1)' + (muw/Pow)*e*(Xw_d*obj.Sec'))';
                    obj.Wc = (mean(Wd,1)' + (muw/Pow)*e*obj.Fd )';
                    Wd     = [obj.Wc; Wd(1:end-1,:)];
                Er(i)  = e ;   
            end          
        end
        function obj= set_Sec(obj, Sec)
            obj.Sec = Sec ;
        end
    end
end