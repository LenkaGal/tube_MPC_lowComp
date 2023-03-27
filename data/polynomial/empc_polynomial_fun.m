function u = empc_polynomial_fun(x)
    global empc a
    order = length(a)-1;

    u = sum(a{end});
    for i = 1:order
        u = u + a{i}*x.^(order+1-i);
    end
    u = 0.5*(0.5*(u + u_max - abs(u - u_max)) + u_min + abs(0.5*(u + u_max - abs(u - u_max)) - u_min));
end