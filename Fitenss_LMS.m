function f_value = Fitenss_LMS(X_before_vector, X_after_vector)
    Pow_begin = 10*log10(var(X_before_vector));
    Pow_end   = 10*log10(var(X_after_vector));
    f_value   = Pow_end - Pow_begin ;
end