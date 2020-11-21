A = imread('ejemplo1(1).jpg'); imshow(A)
[centersDark, radiiDark] = imfindcircles(A,[11 20],'ObjectPolarity','dark');
viscircles(centersDark, radiiDark,'EdgeColor','b');
