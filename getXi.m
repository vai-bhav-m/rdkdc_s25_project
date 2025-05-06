function Xi = getXi(g)
% Input:
%   g - A 4x4 homogeneous transformation matrix in SE(3)
% Output:
%   Xi - A 6x1 unnormalized twist vector [v; w] 

tol = 1e-10;               
R = g(1:3,1:3);            
p = g(1:3,4);              

% Case 1: Pure translation (no rotation)
if norm(R - eye(3), 'fro') < tol
    Xi = [p' 0 0 0]';

% Case 2: General rigid body motion (rotation + translation)
else  
    wcap = logm(R);
    w = [wcap(3,2), wcap(1,3), wcap(2,1)]';
    theta = norm(w); 
    w = w / theta;
    wcap = SKEW3(w);
    v = pinv(theta * eye(3) + (1 - cos(theta)) * wcap + (theta - sin(theta)) * wcap * wcap) * p;
    % Combine into 6D twist vector: [v; w], scaled by theta
    Xi = theta * [v; w];
end