function R = quat2rot(q)
    if length(q) ~= 4
        error('quat2rot: q must be a 4 long vector');
    end

%     q_vec = q(1:3);
%     q_0 = q(4);
%     
%     if q_0 == 0
%         R = eye(3);
%     else
%         theta = 2*acos(q_0);
%         w_hat = skew(q_vec / sin(theta/2));
%         R = expmExact(w_hat, theta);
%     end
    
    R = quat2dcm(q);
end