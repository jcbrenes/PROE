#https://techtutorialsx.com/2019/12/08/python-opencv-detecting-and-decoding-a-qrcode/
#https://docs.opencv.org/4.0.0/de/dc3/classcv_1_1QRCodeDetector.html#a7290bd6a5d59b14a37979c3a14fbf394

import cv2

#Imagen debe estar guardada en la misma carpeta del script

#image = cv2.imread('QRtest.jpg')  #imagen aula
#image = cv2.imread('qrmasalto.jpg') #imagen aula con qr más alto
image = cv2.imread('qrprobe.jpg')  #imagen de prueba genérica

qrCodeDetector = cv2.QRCodeDetector()
 
decodedText, points, _ = qrCodeDetector.detectAndDecode(image)
 
if points is not None:
 
    nrOfPoints = len(points)
 
    for i in range(nrOfPoints):
        nextPointIndex = (i+1) % nrOfPoints
        cv2.line(image, tuple(points[i][0]), tuple(points[nextPointIndex][0]), (255,0,0), 5)
 
    print(decodedText)    
 
    cv2.imshow("Image", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
     
 
else:
    print("QR code not detected")
