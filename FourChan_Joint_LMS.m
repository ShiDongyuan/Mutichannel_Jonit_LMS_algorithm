classdef FourChan_Joint_LMS_Element
	properties
		% 4-control filters 
		Wc_1
		Wc_2 
		Wc_3 
		Wc_4
		Xd_1
		Xd_2
		Xd_3
		Xd_4
		% 4-secondary path estimates
		Sec_1
		Sec_2
		Sec_3
		Sec_4
		Fd_1
		Fd_2
		Fd_3
		Fd_4
		% Lenght of filters
		Len 
		Ls 
	end
	methods
		function obj = FourChan_Joint_LMS_Element(Len
												  ,Ls 
												  ,Sec_1,Sec_2,Sec_3,Sec_4)
			obj.Len   = Len  ; 
			obj.Ls    = Ls   ;
			%-----------------
			obj.Wc_1  = zeros(Len,1); 
			obj.Wc_2  = zeros(Len,1);
			obj.Wc_3  = zeros(Len,1);
			obj.Wc_4  = zeros(Len,1);
			obj.Xd_1  = zeros(Len+Ls,1);
			obj.Xd_2  = zeros(Len+Ls,1);
			obj.Xd_3  = zeros(Len+Ls,1);
			obj.Xd_4  = zeros(Len+Ls,1);
			%-----------------
			obj.Seh_1 = flip(Sec_1);
			obj.Seh_2 = flip(Sec_2);	
			obj.Seh_3 = flip(Sec_3);
			obj.Seh_4 = flip(Sec_4);
			obj.Ed_1  = zeros(Ls,1)
			obj.Ed_2  = zeros(Ls,1)
			obj.Ed_3  = zeros(Ls,1)
			obj.Ed_4  = zeros(Ls,1)
			
		end
		function [y1,y2,y3,y4,obj] = controller(obj, muw, xin_1, xin_2, xin_3, xin_4, e1, e2, e3, e4)
			% muw -- the step size of the algorithm
			% Xin -- the reference signal vector [N x R_num]
			% Dis -- the disturbance vector      [N x E_num]
			obj.Xd_1 = [xin_1; obj.Xd_1(1,end-1)];
			obj.Xd_2 = [xin_2; obj.Xd_2(1,end-1)];
			obj.Xd_3 = [xin_4; obj.Xd_3(1,end-1)];
			obj.Xd_4 = [xin_4; obj.Xd_4(1,end-1)];
			
			obj.Ed_1 = [e1; obj.Ed_1(1,end-1)];
			obj.Ed_2 = [e2; obj.Ed_2(1,end-1)];
			obj.Ed_3 = [e3; obj.Ed_3(1,end-1)];
			obj.Ed_4 = [e4; obj.Ed_4(1,end-1)];
			
			% Feedforward
			y1 = obj.Wc_1'*obj.Xd_1(1:obj.Len);
			y2 = obj.Wc_2'*obj.Xd_2(1:obj.Len);
			y3 = obj.Wc_3'*obj.Xd_3(1:obj.Len);
			y4 = obj.Wc_4'*obj.Xd_4(1:obj.Len);
			
			% Backforward
			ef1 = obj.Seh_1'*obj.Ed_1;
			ef2 = obj.Seh_2'*obj.Ed_2;
			ef3 = obj.Seh_3'*obj.Ed_3;
			ef4 = obj.Seh_4'*obj.Ed_4;
			
			obj.Wc_1 = obj.Wc_1 + muw*(ef1 + ef2 + ef3 + ef4)*obj.Xd_1(obj.Ls:end);
            obj.Wc_2 = obj.Wc_2 + muw*(ef1 + ef2 + ef3 + ef4)*obj.Xd_2(obj.Ls:end);
			obj.Wc_3 = obj.Wc_3 + muw*(ef1 + ef2 + ef3 + ef4)*obj.Xd_3(obj.Ls:end);
			obj.Wc_4 = obj.Wc_4 + muw*(ef1 + ef2 + ef3 + ef4)*obj.Xd_4(obj.Ls:end);
		end
	end
end