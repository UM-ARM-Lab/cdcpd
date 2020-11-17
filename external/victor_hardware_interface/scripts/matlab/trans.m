function [ h ] = trans( p )
%TRANS - creates a 4x4 homogenous transform, with the input as the
% translation, and no rotation

h = eye(4);
h(1:3, 4) = p;

end

