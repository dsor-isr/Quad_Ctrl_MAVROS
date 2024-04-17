function R1 = reshapeRotm(R)
% Put the columns of a rotation matrix into a single row vector.
% 
% JOAO PINTO (2021-09-21)

    R1 = squeeze(R);
    R1 = permute(R1,[3 1 2]);
    R1 = reshape(R1,[],9);
end
