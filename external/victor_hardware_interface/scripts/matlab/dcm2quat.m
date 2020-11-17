function q = dcm2quat(R)        

    Qxx = R(1,1);
    Qxy = R(1,2);
    Qxz = R(1,3);

    Qyx = R(2,1);
    Qyy = R(2,2);
    Qyz = R(2,3);

    Qzx = R(3,1);
    Qzy = R(3,2);
    Qzz = R(3,3);

    k = 1/3.*[Qxx-Qyy-Qzz, Qxy+Qyx, Qxz+Qzx, Qzy-Qyz;
              Qxy+Qyx, Qyy-Qxx-Qzz, Qyz+Qzy, Qxz-Qzx;
              Qxz+Qzx, Qyz+Qzy, Qzz-Qxx-Qyy, Qyx-Qxy;
              Qzy-Qyz, Qxz-Qzx, Qyx-Qxy, Qxx+Qyy+Qzz];
    [v,d] = eig(k);


    [~,i] = max(diag(d));
    i = i(1);

    q = v(:,i);
end