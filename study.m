function [rms_mean, avail_mean] = study(cases)
%STUDY  Mean post-acquisition RMS miss and availability over a set of random cases.
r = zeros(numel(cases),1);  a = r;
for j = 1:numel(cases)
    o = simulate_engagement(cases(j));  r(j) = o.rms_miss;  a(j) = o.avail;
end
rms_mean = mean(r);  avail_mean = mean(a);
end
