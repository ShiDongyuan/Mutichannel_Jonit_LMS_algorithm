function y=FxLMS_B0(S,x_in,e_in,u,reset)
N = 200 ;
L = 100 ;
persistent W Xd Fd ind_x fx
if isempty(W)
    W  = single(zeros(N,1));
end
if isempty(Xd)
    Xd = single(zeros(N,1));
end
if isempty(Fd)
    Fd = single(zeros(N,1));
end
if isempty(ind_x)
    ind_x = uint32(0);
end
if isempty(fx)
    fx    = single(0);
end
if(reset==1)
    W  = single(zeros(N,1));
    Xd = single(zeros(N,1));
    Fd = single(zeros(N,1));
    ind_x = uint32(1);
    fx    = single(0);
else   
for ii=1:N
    
    if(ii==1)
      Xd(ind_x)   = x_in ; 
      Fd(ind_x)   = fx   ;
      y           = single(0);
      fx          = single(0);
    end
    W(ii) = W(ii) + e_in*u*Fd(ind_x);
    y     = y     + W(ii)*Xd(ind_x);
    %%
    if(ii<=L)
       fx = fx + S(ii)*Xd(ind_x); 
    end
    %%
    if(ii<N)
        ind_x = ind_x + 1 ;
        if(ind_x==N+1)
            ind_x = 1 ;
        end
    end
end
end