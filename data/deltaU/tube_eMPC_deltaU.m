function u = tube_eMPC_deltaU(input)
    global ImplicitOptimizer K
    [out, flag] = ImplicitOptimizer{[input(1); input(2)]};
    sprintf('Code Error: %d',flag)
    u = out(1) + K*( input(1) - out(2) );
end