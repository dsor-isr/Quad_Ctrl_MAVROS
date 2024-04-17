function thr = acc2Thr(mass,acc)
% Compute thrust from linear acceleration.
% 
% Input:
% - mass: Vehicle mass in kg,
% 
% - acc: (N x 3) matrix, where each column corresponds to the Cartesian
%        components of the N linear acceleration vectors across the rows of
%        this matrix;
% 
% Output:
% - thr: column vector of length N, where thr(j) corresponds to the value
%        of thrust in newtons obtained from acc(j,:).
% 
% JOAO PINTO (2021-09-21)
    
    acc(:,3) = acc(:,3) + 9.8;
    t = sqrt( sum( acc.^2, 2 ) );
    thr = mass*t;
end

