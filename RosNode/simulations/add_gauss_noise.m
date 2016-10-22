function [output] = add_gauss_noise(mean,variance)
    %drive in a straight line
    noise = -1 + (2).*rand(12,1);
    output = mean + (variance / 6 * sum(noise));
end
