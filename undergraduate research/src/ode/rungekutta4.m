function [t_list, y] = rungekutta4(f, t_span, y0, h)
    t0 = t_span(1);
    tf = t_span(2);

    t_list = t0:h:tf;
    N = length(t_list);

    y = zeros(length(y0), N);
    y(:,1) = y0;

    for i = 1:N-1
        t  = t_list(i);
        yi = y(:,i);

        k1 = f(t, yi);
        k2 = f(t + 0.5*h, yi + 0.5*h*k1);
        k3 = f(t + 0.5*h, yi + 0.5*h*k2);
        k4 = f(t + h,     yi + h*k3);

        y(:,i+1) = yi + (h/6)*(k1 + 2*k2 + 2*k3 + k4);

      
    end
end