function [step_size, Step_size_vector] = Step_size_bound_calculation(Ref, Err_Num, Len_c, Len_s)
    Ref_num          = size(Ref,2)     ;
    Step_size_vector = zeros(Ref_num,1);
    for jj = 1:Ref_num
        Power = rms(Ref(:,jj))^2 ;
        Step_size_vector(jj) = (2/(Err_Num*Len_c*Power))*(1/(2*(2*Len_s+1)));
    end
    step_size = min(Step_size_vector);
end