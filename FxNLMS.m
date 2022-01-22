classdef FxNLMS
    properties
         Wc 
         Xd 
         Sec
         Fd
         len
    end
    methods
        function obj = FxNLMS(N, Sec)
            obj.Wc = zeros(1,N);
            obj.Xd = zeros(N,1);
            obj.Fd = zeros(N,1);
            obj.Sec= Sec       ;
            obj.len= length(Sec);
        end
        function [Er,obj]= controller(obj,muw,xin,dis)
            N  = length(xin)    ;
            Yd = zeros(obj.len,1); 
            Er = zeros(N,1)     ; 
            for i = 1:N
                obj.Xd = [xin(i); obj.Xd(1:end-1)]  ;
                y      =  obj.Wc * obj.Xd           ;
                Yd     = [y;Yd(1:end-1)]            ;
                e      = dis(i) - obj.Sec * Yd      ;
                fx     = obj.Sec * obj.Xd(1:obj.len);
                obj.Fd = [fx;obj.Fd(1:end-1)]       ;
                Pow    = norm(obj.Fd)^2             ;
                if Pow == 0 
                    Pow = 1e-12 ;
                end
                obj.Wc = (obj.Wc' + (muw/Pow)*e*obj.Fd)';
                Er(i)  = e ;   
            end          
        end
        function obj= set_Sec(obj, Sec)
            obj.Sec = Sec ;
        end
    end
end