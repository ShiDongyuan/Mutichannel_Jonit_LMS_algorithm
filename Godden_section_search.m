function Lambda = Godden_section_search(f,a,b,delta)
%% First step.  
% f is fiteness function 
% a is the lower bound  of the lambda. 
% b is the uppder bound of the lambda. 
% delta is the stop condition.

%% Secondary step.
	x_g = a + 0.618*(b-a);
	x_c = a + 0.382*(b-a);
	
	f_g     = f(x_g);
	f_c     = f(x_c);
	C_dist  = b-a   ;
%% Third step 
	while 1
		[Lambda, f_g, f_c, x_g, x_c, a, b] = third_step_function(f_g, f_c, x_g, x_c, a, b, delta, f, C_dist);
		if Lambda ~=0 
			break
		end
	end
end 

function [Lambda, f_g, f_c, x_g, x_c, a, b] = third_step_function(f_g, f_c, x_g, x_c, a, b, delta, f, C_dist)
	
	Lambda  = 0 ;
	
    if f_c <= f_g 
		b   = x_g ;
		
		Crition=(b-a)/C_dist;
        if Crition <= delta	
			Lambda  = (a+b)/2 ;
		else 
			x_g = x_c ;
			x_c = a + 0.382*(b-a); 
			f_g = f(x_g);
			f_c = f(x_c);
        end 
	else
		a = x_c ;
		Crition=(b-a)/C_dist;
        if Crition <= delta	
			Lambda  = (a+b)/2 ;
		else 
			x_c = x_g ;
			x_g = a + 0.618*(b-a);
			f_c = f(x_c);
			f_g = f(x_g);
        end 
    end 
end