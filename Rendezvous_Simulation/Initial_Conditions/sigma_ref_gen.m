function sigma_ref_gen(sigma_t_deg, r_f, r)

sigma_t = deg2rad(sigma_t_deg);
eqn = @(x) sqrt(r_f/r) * cos((sigma_t+x)/2) - cos(x);

init_try = deg2rad(-40);

try
    lead_p = fzero(eqn, init_try);
catch
    lead_p = 0;
end

lead_p_deg = rad2deg(lead_p);

fprintf(">>> lead angle = %.2f degree\n", lead_p_deg);

end