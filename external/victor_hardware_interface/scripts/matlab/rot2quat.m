function q = rot2quat(R)
    if size(R) ~= [3 3]
        error('rot2quat: R must be a 3x3 matrix');
    end

%     if trace(R) == 3
%         theta = 0;
%         w = [1 0 0];
%     else
%         theta = acos((trace(R) - 1)/2);
%         w(1) = R(3,2) - R(2,3);
%         w(2) = R(1,3) - R(3,1);
%         w(3) = R(2,1) - R(1,2);
%         w = w / (2*sin(theta));
%     end
%     
%     q  = -[w*sin(theta/2), cos(theta/2)]';
    
    q = dcm2quat(R);
end