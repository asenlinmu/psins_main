function ang = angle2pi(ang)
% See also angle2c, att2c.
    ang = atan2(sin(ang),cos(ang)); % within [-pi,pi]
