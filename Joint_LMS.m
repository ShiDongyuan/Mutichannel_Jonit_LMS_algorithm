classdef Joint_LMS
	properties
		Wc  % Control filter 
		Sec % Secondary path coefficients [1 x Ls] 
		Len % The length of the control filter
		Ls  % The length of the secondary path
	end
	methods
		%----------------------------------
		function obj = Joint_LMS(Len, Sec)
			obj.Wc  = zeros(1, Len) ; % Control filter [1 x N]
			obj.Len = Len           ;
			obj.Ls  = length(Sec)   ;
			obj.Sec = Sec           ;
		end
		%----------------------------------
		function [Err, obj] = controller(obj,muw,xin,dis)
			N   = length(xin)     ;
			Yd  = zeros(obj.Ls,1) ;
			Err = zeros(N,1)      ;
			Xd  = zeros(obj.Len + obj.Ls,1);
			Ed  = zeros(obj.Ls,1) ;
			obj.Sec = obj.Sec         ;
			Seh = flip(obj.Sec)       ;
			PowS = norm(obj.Sec).^2   ;
			for ii = 1:N
				Xd = [xin(ii);Xd(1:end-1)];
				y  = obj.Wc*Xd(1:obj.Len)     ;
				Yd = [y;Yd(1:end-1)]      ;
				e  = dis(ii)-obj.Sec*Yd       ;
				Ed = [e;Ed(1:end-1)]      ;
				Ef = Seh * Ed             ;
				Pow= norm(Xd(obj.Ls:obj.Ls+obj.Len-1)).^2*PowS;
                if Pow == 0 
                    Pow = 1e-12 ;
                end
				%Pow = 1;
				obj.Wc = (obj.Wc' + (muw/Pow)*Ef*Xd(obj.Ls:obj.Ls+obj.Len-1))';
				Err(ii) = e ;
			end
		end
	end
end