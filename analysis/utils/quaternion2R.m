function R = quaternion2R(Q)
% Obatain rotation matrices from a set of quaternions.
% 
% Input:
% - Q: (N x 4) matrix, where each row is a quaternion.
% 
% Output:
% - R: (3 x 3 x N) tensor, where R(:,:,j) is the rotation matrix
%      obtained from the quaternion Q(j,:).
% 
% JOAO PINTO (2021-09-21)

    len = size(Q,1);
    A = zeros(3,3);
    R = zeros(3,3,len);

    for j = 1:len
        
        D = 1 - 2*[Q(j,3)^2 + Q(j,4)^2; 
                   Q(j,2)^2 + Q(j,4)^2; 
                   Q(j,2)^2 + Q(j,3)^2];
        
        a = Q(j,2)*Q(j,3);
        b = Q(j,1)*Q(j,4);

        A(1,2) = a - b;
        A(2,1) = a + b;

        c = Q(j,2)*Q(j,4);
        d = Q(j,3)*Q(j,1);

        A(1,3) = c + d;
        A(3,1) = c - d;

        e = Q(j,3)*Q(j,4);
        f = Q(j,2)*Q(j,1);

        A(2,3) = e - f;
        A(3,2) = e + f;

        R(:,:,j) = diag(D) + 2*A;
        
    end
    
end
