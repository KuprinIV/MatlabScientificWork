function out_vector = decimate(input_vector, decimate_factor)
    decim_cntr = 1;
    sum = 0;
    out_vector = zeros(floor(length(input_vector)/decimate_factor), 1);
    for i=1:length(input_vector)
        sum = sum + input_vector(i);
        if(mod(i, decimate_factor) == 0)
            out_vector(decim_cntr, 1) = sum/decimate_factor;
            decim_cntr = decim_cntr+1;
            sum = 0;
        end
    end
end