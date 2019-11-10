function xx = ResampleConfigSimple(x, NE)
[nv, np, nfe] = size(x);
sss = [];
LARGE_NUM = 100;

for ii = 1 : nv
    for jj = 1 : np
        temp_x = [];
        for kk = 1 : (nfe - 1)
            temp = linspace(x(ii,jj,kk), x(ii,jj,kk+1), LARGE_NUM);
            temp_x = [temp_x, temp(1,1 : (LARGE_NUM - 1))];
        end
        temp_x = [temp_x, x(ii, jj, nfe)];
        index = round(linspace(1, length(temp_x), NE));
        sss = [sss, temp_x(index)];
    end
end

counter = 0;
for ii = 1 : nv
    for jj = 1 : np
        for kk = 1 : NE
            counter = counter + 1;
            xx(ii, jj, kk) = sss(counter);
        end
    end
end
end