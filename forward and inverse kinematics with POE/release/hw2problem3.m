    % RBE 501 - Robot Dynamics - Spring 2022
% Homework 2, Problem 3
% Worcester Polytechnic Institute
%
% Instructor: L. Fichera <lfichera@wpi.edu>
% Last modified: 02/06/2022
clear, clc, close all
addpath('utils');

%% Create the manipulator
n = 6; % degrees of freedom
plotOn = false;
nTests = 10; % number of random test configurations
TestI =30;
% Robot dimensions (meters)
H1 = 0.320;
H2 = 0.225;
H3 = 0.225;
H4 = 0.065;
W  = 0.035;

robot = SerialLink([Revolute('d', H1, 'a', 0,  'alpha', -pi/2, 'offset', 0), ...
    Revolute('d', 0,  'a', H2, 'alpha', 0,     'offset', -pi/2), ...
    Revolute('d', W,  'a', 0,  'alpha', pi/2,  'offset', pi/2), ...
    Revolute('d', H3, 'a', 0,  'alpha', -pi/2, 'offset', 0), ...
    Revolute('d', 0,  'a', 0,  'alpha', pi/2,  'offset', 0), ...
    Revolute('d', H4, 'a', 0,  'alpha', 0,     'offset', 0)], ...
    'name', 'Staubli TX-40');

% Joint limits
qlim = [-180  180;  % q(1)
    -125  125;  % q(2)
    -138  138;  % q(3)
    -270  270;  % q(4)
    -120  133.5;% q(5)
    -270  270]; % q(6)

% Display the manipulator in the home configuration
q = zeros(1,n);
robot.teach(q);


%% Part A - Calculate the screw axes
% Let us calculate the screw axis for each joint
% Put all the axes into a 6xn matrix S, where n is the number of joints
    S1 = transpose([0 0 1 0 0 0]);
    S2 =  transpose([0 1 0 -.32 0 0]);
   % S3= transpose([1 0 0 0.3 0 0]);
   
   S3= transpose([0 1 0 -.545 0 0]);
 S4= transpose([0 0 1 .035 0 0]);
  S5= transpose([0 1 0 -.77 0 0]);
   S6= transpose([0 0 1 0.035 0 0]);
    S= [S1 S2 S3 S4 S5 S6]


%% Part B - Calculate the forward kinematics with the Product of Exponentials formula
% First, let us calculate the homogeneous transformation matrix M for the
% home configuration
R = [1 0 0 ; 0 1 0; 0 0 1];
P = transpose([0 0.035 .835]);
M = [R P ; 0 0 0 1];

fprintf('---------------------Forward Kinematics Test---------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%'); 
 
% Test the forward kinematics for 100 random sets of joint variables
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Generate a random configuration
    q = [qlim(1,1) + (qlim(1,2) - qlim(1,1)) * rand(), ...
         qlim(2,1) + (qlim(2,2) - qlim(2,1)) * rand(), ...
         qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand(),...
         qlim(4,1) + (qlim(4,2) - qlim(4,1)) * rand(), ...
qlim(5,1) + (qlim(5,2) - qlim(5,1)) * rand(), ...
qlim(6,1) + (qlim(6,2) - qlim(6,1)) * rand(), ...
];
    
    % Calculate the forward kinematics
    T = fkine(S,M,q)
    a= robot.fkine(q)
    if plotOn
        robot.teach(q);
        title('Forward Kinematics Test');
    end
    a=robot.fkine(q);
    assert(all(all(abs(double(robot.fkine(q)) - T) < 1e-10)));
end
 
fprintf('\nTest passed successfully.\n');


%% Part C - Calculate the Space Jacobian of the manipulator
fprintf('-------------------Differential Kinematics Test------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%'); 

% Test the correctness of the Jacobian for 100 random sets of joint
% variables
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Generate a random configuration
     q = [qlim(1,1) + (qlim(1,2) - qlim(1,1)) * rand(), ...
         qlim(2,1) + (qlim(2,2) - qlim(2,1)) * rand(), ...
         qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand(),...
         qlim(4,1) + (qlim(4,2) - qlim(4,1)) * rand(), ...
qlim(5,1) + (qlim(5,2) - qlim(5,1)) * rand(), ...
qlim(6,1) + (qlim(6,2) - qlim(6,1)) * rand(), ...
];
    % Calculate the Forward Kinematics
    T = fkine(S,M,q);
    
    % Calculate the Jacobian
    J = jacob0(S,q);
    
    if plotOn
        robot.teach(q);
        title('Differential Kinematics Test');
    end
    
    % Test the correctness of the Jacobian
    Jcoords = [-skew(T(1:3,4))*J(1:3,:)+J(4:6,:); J(1:3,:)];
    assert(all(all(abs(double(robot.jacob0(q)) - Jcoords) < 1e-10)));
end

fprintf('\nTest passed successfully.\n');

%% Part D - Inverse Kinematics
fprintf('----------------------Inverse Kinematics Test--------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%');

% Calculate the twist representing the robot's home pose
currentPose = MatrixLog6(M);
currentPose = [currentPose(3,2) currentPose(1,3) currentPose(2,1) currentPose(1:3,4)']';

% Set the current joint variables
currentQ = zeros(1,6);

if plotOn
    robot.teach(currentQ);
    h = triad('matrix', M, 'tag', 'Target Pose', 'linewidth', 2.5, 'scale', 0.5);
end
     
% Generate the test configurations
q = [linspace(0,pi/2,TestI);
     linspace(0,pi/6,TestI);
     linspace(0,pi/6,TestI);
     linspace(0,pi/2,TestI);
     linspace(0,pi/3,TestI);
     linspace(0,pi/2,TestI)];

for ii = 1 : nTests

    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    fprintf('DONE\n')
    % Generate the robot's pose
    T = fkine(S,M,q(:,ii)');
    targetPose = MatrixLog6(T);
    targetPose = [targetPose(3,2) targetPose(1,3) targetPose(2,1) targetPose(1:3,4)']';
    
    if plotOn
        set(h, 'matrix', T);
        title('Inverse Kinematics Test');
        drawnow;
    end
    
    % Inverse Kinematics
    counter=0;
%     while counter < 100
%         counter= counter+1;
%         J = jacob0(S,currentQ); 
%         error = targetPose - currentPose;
%         delta = transpose(J);
%         deltaQ = delta*error;
%         fprintf("insidde first")
%         currentQ = currentQ + deltaQ'
% 
%     end

    while norm(targetPose - currentPose) > 1e-3
        disp(q(:,ii)')
        J = jacob0(S,currentQ); 
        error = targetPose - currentPose;
        lamda = 1 - robot.maniplty(currentQ);

        delta = transpose(J)* pinv(J*transpose(J) + lamda^2 * eye(6));
        deltaQ = delta*error;
      
        currentQ = currentQ + deltaQ';
        
        T = fkine(S,M,currentQ);
        currentPose = MatrixLog6(T);
        currentPose = [currentPose(3,2) ...
                       currentPose(1,3) ...
                       currentPose(2,1) ...
                       currentPose(1:3,4)']';
        
        if plotOn
            try
                robot.teach(currentQ);
                drawnow;
            catch e
                continue;
            end
        end
    end
end

fprintf('\nTest passed successfully.\n');
