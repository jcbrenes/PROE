clear all;
close all;

%File=  "CUARTO2.TXT";
%File=  "MESA2.TXT";
%File=  "SALA3.TXT";
File= "LATERAL3.TXT";
Datos= importdata (File," ");

%Calcula la diferencia en orientación entre los mags
DeltaPhi= Datos(:,4) - Datos(:,5);

%Definir el tamano a usar en los graficos
InicioTrend=5;
FinTrend= size(Datos(:,1))(1,1)-5;

AfPBH=Datos(:,3);
DPfPBH=zeros(FinTrend,1);
AfPBL=Datos(:,3);
DPfPBL=zeros(FinTrend,1);

%Aplica el primer filtro paso bajo (HIGH) (el de filtrar el ruido)
alpha1=0.5  %valor manual puesto en fc=fsampling
for i = 2:FinTrend
  AfPBH(i)= (1-alpha1)*AfPBH(i-1) + (alpha1) * Datos(i,3);
  DPfPBH(i)= (1-alpha1)*DPfPBH(i-1) + (alpha1) * DeltaPhi(i);
endfor

%Aplica el segundo filtro paso bajo (LOW) (el de filtrar las detecciones y solo dejar el valor base)
alpha2=0.0231
for i = 2:FinTrend
  AfPBL(i)= (1-alpha2)*AfPBL(i-1) + (alpha2) * Datos(i,3);
  DPfPBL(i)= (1-alpha2)*DPfPBL(i-1) + (alpha2) * DeltaPhi(i);
endfor

%Resta el valor base (segundo filtro) al valor sin ruido (primer filtro)
Af= AfPBH - AfPBL;
DPf = DPfPBH - DPfPBL;

%convierte uno de los vectores de tiempo a segundos
TiempoA= Datos(:,1)/1000000;

hold
%plot(TiempoA(InicioTrend:FinTrend), AfPBH(InicioTrend:FinTrend),";FPBH;");
%plot(TiempoA(InicioTrend:FinTrend), AfPBL(InicioTrend:FinTrend),";FPBL;");
plot(TiempoA(InicioTrend:FinTrend), Af(InicioTrend:FinTrend),";Filter;");
title('Eje Z mag HSC')
xlabel('Time (s)')
ylabel('Magnetic field')

figure
hold
%plot(TiempoA(InicioTrend:FinTrend), DPfPBH(InicioTrend:FinTrend),";FPBH;");
%plot(TiempoA(InicioTrend:FinTrend), DPfPBL(InicioTrend:FinTrend),";FPBL;");
plot(TiempoA(InicioTrend:FinTrend), DPf(InicioTrend:FinTrend),";Filter;");
title('Delta Phi (Dif Orientacion)')
xlabel('Time (s)')
ylabel('Grados')

