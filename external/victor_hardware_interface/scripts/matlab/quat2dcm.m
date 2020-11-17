function R = quat2dcm(q)

    w=q(4);
    x=q(1);
    y=q(2);
    z=q(3);

    Rxx = 1 - 2*(y^2 + z^2);
    Rxy = 2*(x*y - z*w);
    Rxz = 2*(x*z + y*w);

    Ryx = 2*(x*y + z*w);
    Ryy = 1 - 2*(x^2 + z^2);
    Ryz = 2*(y*z - x*w );

    Rzx = 2*(x*z - y*w );
    Rzy = 2*(y*z + x*w );
    Rzz = 1 - 2 *(x^2 + y^2);

    R = [Rxx, Rxy, Rxz;
         Ryx, Ryy, Ryz;
         Rzx, Rzy, Rzz];

end