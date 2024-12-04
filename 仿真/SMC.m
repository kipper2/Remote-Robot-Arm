S_flat = 4 * norm(x,2)^(0.5)*sign(x) - integral(-4*sign(x));

S_Flat_deriv = -4*sign(x) +4*(0.5*norm(x,2)^(0.5) * (x_deriv*x)^(0.5));

u = -S_Flat_deriv - sign(S_flat) - S_flat;