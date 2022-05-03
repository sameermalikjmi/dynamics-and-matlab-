function T = twist2ht(S,theta)
    omega = transpose([S(1) S(2) S(3)]);
    R = axisangle2rot(omega,theta);
    skew = skewSy(omega);
    next = eye(3)*theta + (1 -cos(theta))*skew;
    next = next + (theta -sin(theta))*(skew*skew);
    v=  transpose([S(4) S(5) S(6)]);
    next = next*v;
    T= [R next; 0 0 0 1];


end

function skew = skewSy(omega)
    skew = [0 -omega(3) omega(2); omega(3) 0 -omega(1); -omega(2) omega(1) 0];
    
end