function [DTM2] = noise_dtm(DTM, magnitude)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
alpha = 0.1;

Noise = zeros(size(DTM));
Noise(1,1) = randn();

for j=2:size(DTM,1)
    Noise(j,1) = (1-alpha)*Noise(j-1,1) + alpha*randn();
end

for k=2:size(DTM,2)
    Noise(1,k) = (1-alpha)*Noise(1,k-1) + alpha*randn();
end

for j=2:size(DTM,1)
    for k=2:size(DTM,2)
        Noise(j,k) = (1-alpha)/2*Noise(j,k-1) + ...
                     (1-alpha)/2*Noise(j-1,k) + ...
                     alpha*randn();
    end
end

DTM2 = DTM + magnitude*Noise;

end

