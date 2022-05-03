function Vtrans = adjoint(V,T)
 R= T(1:3,1:3);
    p = T(1:3,4);
    skew_p = skew(p);
    res = skew_p*R;
    zero = zeros(3);
    Tr = [R zero; res R];
    Vtrans = Tr*V;  
end

