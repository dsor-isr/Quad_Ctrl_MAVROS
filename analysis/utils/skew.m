function X = skew(x)
% Obtain 3-D array of skew-symmetric matrices from Nx3 matrix
% 
% JOAO PINTO (2021-09-21)

    s = size(x,1);
    X = zeros(3,3,s);

    for j = 1:s
        
        X(:,:,j) = [   0      -x(j,3)    x(j,2)
                     x(j,3)      0      -x(j,1)
                    -x(j,2)    x(j,1)      0     ];    
        
    end

end