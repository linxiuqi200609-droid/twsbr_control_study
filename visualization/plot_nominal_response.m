function figure_handle = plot_nominal_response(raw)
%PLOT_NOMINAL_RESPONSE Plot five-controller response to the 0.75 m step.
results = validate_figure_raw(raw, "S2_position_step_0p75m");
figure_handle = plot_response_axes(results, "Nominal response");
end
