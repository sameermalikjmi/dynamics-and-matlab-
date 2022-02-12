function R = axisangle2rot(omega,theta)
    % your code here
    I = eye(3);
    K = skewSymeetric(omega)
    
    R = I+ sin(theta)*(K)
    Ksquare = (K*K)
    cosva= 1- cos(theta);
    R= R + (cosva.*Ksquare);

end



function T = twist2ht(S,theta)
    % your code here
    
    % If needed, you can calculate a rotation matrix with:
    omega = transpose([S(1) S(2) S(3)]);
    R = axisangle2rot(omega,theta);
    skew = skewSymeetric(omega);
    next = eye(3)*theta + (1 -cos(theta))*skew;
    next = next + (theta -sin(theta))*(skew*skew);
    v=  transpose([S(4) S(5) S(6)]);
    next = next*v;
    T= [R next; 0 0 0 1]
end
function skew = skewSymeetric(omega)
    skew = [0 -omega(3) omega(2); omega(3) 0 -omega(1); -omega(2) omega(1) 0];
    
end


function T = fkine(S,M,q)
   
    % your code here
    S= transpose(S)
    S_number= size(S)
    s=S_number(1)
    multiply = eye(4);
    for i = 1:s
        theta = q(i)
        skew = S(i,:)
        T= twist2h(skew,theta);
        multiply = multiply*T
        
    end
    T= multiply*M 
        
    

end

function T = twist2h(S,theta)
    % your code here
    
    % If needed, you can calculate a rotation matrix with:
    omega = transpose([S(1) S(2) S(3)]);
    R = axisangle2rot(omega,theta);
    skew = skewSymeetric(omega);
    next = eye(3)*theta + (1 -cos(theta))*skew;
    next = next + (theta -sin(theta))*(skew*skew);
    v=  transpose([S(4) S(5) S(6)]);
    next = next*v;
    T= [R next; 0 0 0 1]
end




function Vtrans = adjoint(V,T)
    % your code here
    
    R= T(1:3,1:3)
    p = T(1:3,4)
    skew_p = skewSymeetric(p)
    res = skew_p*R
    zero = zeros(3)
    Tr = [R zero; res R]
    Vtrans = Tr*V  
    
end

function J = jacob0(S,q)
    % your code here
    sn= size(S);
    sn= sn(2);
    J =[];
    multiply = eye(4);
    for i = 1:sn
        si= S(:,i);
        omega = q(i);
        J_i = adjoint(si,multiply)
        J=[J,J_i]
        multiply= multiply*twist2ht(si,omega)
        
        
        
    % If necessary, you calculate the homogeneous transformation associated with a twist V and displacement omega with:
    % T = twist2ht(V,omega);
    
    % You can also calculate the adjoint transformation of a twist V w.r.t. a homogeneous transformation matrix T with:
    % adjoint(V,T)
end
