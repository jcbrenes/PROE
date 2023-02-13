%Script para prueba de lectura de códigos QR o de barras
%Tomado de: https://la.mathworks.com/help/vision/ug/localize-and-read-multiple-barcodes-in-image.html
%Documentación adicional: https://la.mathworks.com/help/vision/ref/readbarcode.html
%For a successful detection, the barcode must be clearly visible. 
%The barcode must also be as closely aligned to a horizontal or vertical position
%as possible. The readBarcode function is inherently more robust to rotations 
%for 2-D or matrix codes than it is to 1-D or linear barcodes. %
%For example, the barcode cannot be detected in this image (Ver NotFoundBarcode)
%Rotate the image using the imrotate so that the barcode is roughly horizontal.
%Use readBarcode on the rotated image.
%Introduced in Matlab R2020a
% Analizador de imágenes de prueba: busca código en las imágenes:
%https://online-barcode-reader.inliteresearch.com/

%Matlab online de prueba:
%https://matlab.mathworks.com/?trial=true&elqsid=1615835338455&potential_use=Student
%Barcode

%The readBarcode function detects only a single barcode in each image.
%In order to detect multiple barcodes, you must specify a region-of-interest 
%(ROI). To specify an ROI, you can use the drawrectangle function to interactively
%determine the ROIs. You can also use image analysis techniques to detect 
%the ROI of multiple barcodes in the image.

%Without preprocessing, barcodes cannot be detected in the image containing
%multiple rotated barcodes.
%Se binariza, se automatizan las ROIs, se usa transformada de Hough


%Prueba con código que sirve
I = imread('barcode1D.jpeg');

% Read the 1-D barcode and determine the format..
[msg, format, locs] = readBarcode(I);

% Display the detected message and format.
disp("Detected format and message: " + format + ", " + msg)
