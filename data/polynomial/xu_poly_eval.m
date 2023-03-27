function p = xu_poly_eval(a, x, order, umin, umax)

p = sum(a{end});
for i = 1:order,
    p = p + a{i}*x.^(order+1-i);
end
if nargin > 3
%     p = max(min(p, umax), umin);
    p = 0.5*(0.5*(p + umax - abs(p - umax)) + umin + abs(0.5*(p + umax - abs(p - umax)) - umin));
end
