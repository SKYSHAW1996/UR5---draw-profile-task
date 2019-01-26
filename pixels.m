m = '0.jpg';
image = profile(m);
imshow(image)
[row,col] = size(image);
S = size(find(image==0))
i = 0;
for r = 1:row
    for c = 89:col
        if image(r,c) == 0
        i = i+1;
        end
    end
end
