function xdot = odeTwoBody(t,x,mu)
  
    % input
    r = x(1:2);
    v = x(3:4);
    rmag = norm(r);

    % derivative of state vector
    rdot = v;
    vdot = - mu / rmag^3 * r;
    xdot = [rdot ; vdot];
    
end