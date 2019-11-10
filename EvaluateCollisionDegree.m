function val = EvaluateCollisionDegree()
load x.txt
load y.txt
yy = zeros(3,4,101);
xx = zeros(3,4,101);
counter = 0;
for ii = 1 : 3
    for jj = 1 : 4
        for kk = 1 : 101
            counter = counter + 1;
            xx(ii,jj,kk) = x(counter);
            yy(ii,jj,kk) = y(counter);
        end
    end
end

global OC Nobs
cur_largest = 0;
cur_v1 = 0;
cur_v2 = 0;

for ii = 1 : 2
    for jj = (ii+1) : 3
        for ind1 = 1 : 4
            for ind2 = 1 : 4
                temp_val = 0;
                for iii = 1 : 101
                    xc1 = xx(ii, ind1, iii);
                    yc1 = yy(ii, ind1, iii);
                    xc2 = xx(jj, ind2, iii);
                    yc2 = yy(jj, ind2, iii);
                    temp_val = temp_val + MeasureCollision(xc1, yc1, xc2, yc2);
                end
                if (temp_val > cur_largest)
                    cur_largest = temp_val;
                    cur_v1 = (ii * 10 + ind1) / 10;
                    cur_v2 = (jj * 10 + ind2) / 10;
                end
            end
        end
    end
end
val = [cur_v1, cur_v2];

cur_largest = 0;
cur_v1 = 0;
cur_v2 = 0;

for ii = 1 : 3
    for jj = 1 : 4
        for kk = 1 : Nobs
            temp_val = 0;
            for iii = 1 : 101
                xc1 = xx(ii, jj, iii);
                yc1 = yy(ii, jj, iii);
                xc2 = OC{kk}.x(iii);
                yc2 = OC{kk}.y(iii);
                temp_val = temp_val + MeasureCollision(xc1, yc1, xc2, yc2);
            end
            if (temp_val > cur_largest)
                cur_largest = temp_val;
                cur_v1 = (ii * 10 + jj) / 10;
                cur_v2 = kk;
            end
        end
    end
end
val = [val, cur_v1, cur_v2];
end

function val = MeasureCollision(xc1, yc1, xc2, yc2)
aa = (xc1 - xc2)^2 + (yc1 - yc2)^2 - 7.9999;
if (aa >= 0)
    val = 0;
else
    val = -aa;
end
end