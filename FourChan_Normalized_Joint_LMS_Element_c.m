%%
%   _   _                            _ _                _   __  __            _       _       _     _     __  __  ____
%  | \ | | ___  _ __ _ __ ___   __ _| (_) ____ ___   __| | |  \/  | ___      | | ___ (_)_ __ | |_  | |   |  \/  |/ ___|
%  |  \| |/ _ \| '__| '_ ` _ \ / _` | | ||_  // _ \ / _` | | |\/| |/ __|  _  | |/ _ \| | '_ \| __| | |   | |\/| |\___ \
%  | |\  | (_) | |  | | | | | | (_| | | | / /|  __/| (_| | | |  | | (__  | |_| | (_) | | | | | |_  | |___| |  | | ___) |
%  |_| \_|\___/|_|  |_| |_| |_|\__,_|_|_|/___|\___| \__,_| |_|  |_|\___|  \___/ \___/|_|_| |_|\__| |_____|_|  |_||____/
%
%%
classdef FourChan_Normalized_Joint_LMS_Element_c
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
		Seh_1
		Seh_2
		Seh_3
		Seh_4
		Sec_1
		Sec_2
		Sec_3
		Sec_4
		Ed_1
		Ed_2
		Ed_3
		Ed_4
		% Lenght of filters
		Len
		Ls
		Yd
	end
	methods
		function obj = FourChan_Normalized_Joint_LMS_Element_c(Len,Ls,Sec_1,Sec_2,Sec_3,Sec_4)
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
			obj.Sec_1 = Sec_1      ;
			obj.Sec_2 = Sec_2      ;
			obj.Sec_3 = Sec_3      ;
			obj.Sec_4 = Sec_4      ;
			obj.Ed_1  = zeros(Ls,1);
			obj.Ed_2  = zeros(Ls,1);
			obj.Ed_3  = zeros(Ls,1);
			obj.Ed_4  = zeros(Ls,1);
			obj.Yd    = zeros(Ls,1);

		end
		function [yt_1,yt_2,yt_3,yt_4,obj] = controller(obj, muw, xin_1, xin_2, xin_3, xin_4, e1, e2, e3, e4)
			% muw -- the step size of the algorithm
			% Xin -- the reference signal vector [N x R_num]
			% Dis -- the disturbance vector      [N x E_num]
			obj.Xd_1 = [xin_1; obj.Xd_1(1:end-1)];
			obj.Xd_2 = [xin_2; obj.Xd_2(1:end-1)];
			obj.Xd_3 = [xin_3; obj.Xd_3(1:end-1)];
			obj.Xd_4 = [xin_4; obj.Xd_4(1:end-1)];

			obj.Ed_1 = [e1; obj.Ed_1(1:end-1)];
			obj.Ed_2 = [e2; obj.Ed_2(1:end-1)];
			obj.Ed_3 = [e3; obj.Ed_3(1:end-1)];
			obj.Ed_4 = [e4; obj.Ed_4(1:end-1)];

			% Feedforward
			y1 = obj.Wc_1'*obj.Xd_1(1:obj.Len);
			y2 = obj.Wc_2'*obj.Xd_2(1:obj.Len);
			y3 = obj.Wc_3'*obj.Xd_3(1:obj.Len);
			y4 = obj.Wc_4'*obj.Xd_4(1:obj.Len);
			yo = y1+y2+y3+y4               ;

			obj.Yd = [yo; obj.Yd(1:end-1)] ;
			yt_1   = obj.Sec_1'*obj.Yd     ;
			yt_2   = obj.Sec_2'*obj.Yd     ;
			yt_3   = obj.Sec_3'*obj.Yd     ;
			yt_4   = obj.Sec_4'*obj.Yd     ;

			% Backforward
			ef1 = obj.Seh_1'*obj.Ed_1;
			ef2 = obj.Seh_2'*obj.Ed_2;
			ef3 = obj.Seh_3'*obj.Ed_3;
			ef4 = obj.Seh_4'*obj.Ed_4;

			obj.Wc_1 = obj.Wc_1 + muw*(ef1 + ef2 + ef3 + ef4)*obj.Xd_1(obj.Ls:end-1)/(norm(obj.Xd_1(obj.Ls:end-1))^2+realmin);
            obj.Wc_2 = obj.Wc_2 + muw*(ef1 + ef2 + ef3 + ef4)*obj.Xd_2(obj.Ls:end-1)/(norm(obj.Xd_2(obj.Ls:end-1))^2+realmin);
			obj.Wc_3 = obj.Wc_3 + muw*(ef1 + ef2 + ef3 + ef4)*obj.Xd_3(obj.Ls:end-1)/(norm(obj.Xd_3(obj.Ls:end-1))^2+realmin);
			obj.Wc_4 = obj.Wc_4 + muw*(ef1 + ef2 + ef3 + ef4)*obj.Xd_4(obj.Ls:end-1)/(norm(obj.Xd_4(obj.Ls:end-1))^2+realmin);
		end
	end
end