function mse = immse(v1, v2)
    mse = 0;
    len = length(min(v1, v2));
    for i = 1:len
       mse = mse + (v1(i)-v2(i))^2; 
    end
    mse = mse/len;
end