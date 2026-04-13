function xdot = odeTwoBody(t,x,mu)
  
    % input
    r = x(1:3);
    v = x(4:6);
    rmag = norm(r);

    % derivative of state vector
    rdot = v;
    vdot = - mu / rmag^3 * r;
    xdot = [rdot ; vdot];
    
end