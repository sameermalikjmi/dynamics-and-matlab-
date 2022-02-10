function T = trot(theta, axis)
    %% your code here
    if(axis == 'z')
        T = [ cos(theta) -sin(theta) 0 ; sin(theta) cos(theta) 0; 0 0 1]
    elseif(axis == 'y')
        T = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)]
    else
        T= [1 0 0; 0 cos(theta) -sin(theta);0 sin(theta) cos(theta)]
        
end



function T = tdh(theta, d, a, alpha)
    %% your code here
    T = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);sin(theta), cos(theta)*cos(alpha), - cos(theta)*sin(alpha), a*sin(theta);0, sin(alpha) ,cos(alpha), d;0, 0, 0, 1]
end


function T = fwkinscara(q)
    % q is a 4x1 vector containing the generalized joint variables
    L1 = 0.5; % [m]
    L2 = 0.5; % [m]
    L4 = 0.1; % [m]
    t1= tdh(q(1),0, L1,0);
    t2= tdh(q(2),0,L2, pi);
    t3= tdh(0,q(3),0,pi);
    t4 = tdh(q(4),-L4,0, pi);
    T= t1*t2
    T= T*t3
    T= T*t4
    % your code here
end
function T = tdh(theta, d, a, alpha)
    %% your code here
    T = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);sin(theta), cos(theta)*cos(alpha), - cos(theta)*sin(alpha), a*sin(theta);0, sin(alpha) ,cos(alpha), d;0, 0, 0, 1];
end

function T = fwkinrpp(q)
   
    L1 = 0.5; % [m]
    % your c
   
    t1= tdh(q(1) -pi/2,L1, 0,0);
    t2= tdh(0,q(2),0, -pi/2);
    t3= tdh(pi/2,q(3),0,0);
    
    T= t1*t2
    T= T*t3
   
    % your code here
end
function T = tdh(theta, d, a, alpha)
    %% your code here
    T = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);sin(theta), cos(theta)*cos(alpha), - cos(theta)*sin(alpha), a*sin(theta);0, sin(alpha) ,cos(alpha), d;0, 0, 0, 1];

end
