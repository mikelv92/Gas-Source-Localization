meanMap = csvread('meanMap.csv');
varMap = csvread('varianceMap.csv');
x_low = 1701;
x_high = 2400;
y_low = 1851;
y_high = 2550;


occupancyMap = im2double(imread('map.pgm'));

finalMap = zeros(700, 700);

for ii = 1:700
    for jj = 1:700
        if occupancyMap(x_low + ii, y_low + jj) < 0.9
            finalMap(ii, jj) = -1;
        else
            finalMap(ii, jj) = meanMap(x_low + ii, y_low + jj);
        end
    end
end

finalColouredMap = uint8(zeros(700, 700, 3));

cm = jet(100);

for ii = 1:700
    for jj = 1:700
        if finalMap(ii,jj) == -1
            finalColouredMap(ii, jj, 1) = 0.8078 * 255;
            finalColouredMap(ii, jj, 2) = 0.8078 * 255;
            finalColouredMap(ii, jj, 3) = 0.8078 * 255;
        else            
            q = floor(1 + (finalMap(ii, jj) - 0) / (0.04055 - 0) * (100 - 1));
            myColor = cm(q, :);
            finalColouredMap(ii, jj, 1) = myColor(1) * 255;            
            finalColouredMap(ii, jj, 2) = myColor(2) * 255;
            finalColouredMap(ii, jj, 3) = myColor(3) * 255;
        end
    end
end
figure, imshow(finalColouredMap, []);