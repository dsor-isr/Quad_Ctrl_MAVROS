function lbd = zxy_R2Euler(R)
% 
%   R in SO(3)
%   lbd in R(3) is the vector of the Z-X-Y Euler angles that define R
%
%   JOAO PINTO (2021-09-21)

    len = size(R,3);
    lbd = zeros(len,3);

    for j = 1:len
        
        lbd(j,1) = atan2(R(3,2,j),sqrt(R(3,1,j)^2+R(3,3,j)^2)); % Roll

        if abs(cos(lbd(j,1))) < eps %if it is a singular configuration
            lbd(j,3) = 0; % Yaw
            lbd(j,2) = sign(lbd(j,1))*atan2(R(1,3,j),R(1,1,j)); % Pitch
        else
            lbd(j,2) = atan2(-R(3,1,j)/cos(lbd(j,1)),R(3,3,j)/cos(lbd(j,1)));
            lbd(j,3) = atan2(-R(1,2,j)/cos(lbd(j,1)),R(2,2,j)/cos(lbd(j,1)));
        end
    
    end
        
end

