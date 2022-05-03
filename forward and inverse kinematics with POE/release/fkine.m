function T = fkine(S,M,q)
   
   S_number= size(S);
   s=S_number(2);
   multiply = eye(4);
   for i = 1:s
        theta = q(i);
        skew = S(:,i);
        T= twist2ht(skew,theta);
        multiply = multiply*T;
        
    end
    T= multiply*M;
end