function J = jacob0(S,q) 
sn= size(S);
    sn= sn(2);
    J =[];
    multiply = eye(4);
    for i = 1:sn
        si= S(:,i);
        omega = q(i);
        J_i = adjoint(si,multiply);
        J=[J,J_i];
        multiply= multiply*twist2ht(si,omega);
end