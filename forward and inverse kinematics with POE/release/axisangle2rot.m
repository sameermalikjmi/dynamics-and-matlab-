function R = axisangle2rot(omega,theta)
    % your code here
    I = eye(3);
    K = skew(omega);
    
    R = I+ sin(theta)*(K);
    Ksquare = (K*K);
    cosva= 1- cos(theta);
    R= R + (cosva.*Ksquare);
end

 % Make sure that omega is a unit vector
 

