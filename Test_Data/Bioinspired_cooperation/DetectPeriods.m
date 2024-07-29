clear all;
close all;
%Importa datos de respuesta del sensor de archivo txt de Jose Pablo
%File=  "CUARTO6.TXT";
%File=  "MESA2.TXT";
%File=  "SALA3.TXT";
%Datos= importdata (File," ");

eventos=[];
%archivo= {"CUARTO1.TXT";"CUARTO2.TXT";"CUARTO3.TXT";"CUARTO4.TXT";"CUARTO5.TXT";"CUARTO6.TXT"};
%archivo= {"MESA1.TXT";"MESA2.TXT";"MESA3.TXT";"MESA4.TXT";"MESA5.TXT";"MESA6.TXT"};
archivo= {"SALA1.TXT";"SALA2.TXT";"SALA3.TXT";"SALA4.TXT";"SALA5.TXT";"SALA6.TXT"};
for j=1:6
  clear tpico;
  clear fpico;
  clear Datos;
  clear tini;
  clear tfin;

  File=char(archivo(j));
  Datos= importdata (File," ");

  %Busca tiempo de sampling
  deltat=diff(Datos(:,1)); %Encuentra los deltas de tiempo
  tsampling= mean(deltat)/1000; %Promedio en ms
  fsampling= 1/(tsampling/1000);

  %Convierte vectores de tiempo
  TiempoA= Datos(:,1)/1000000; %convierte el vector de tiempo a segundos
  Tiempoms= Datos(:,1)/1000; %convierte el vector de tiempo a milisegundos


  %Aplicacion de filtro media movil a los datos
  valorFiltroH=1;
  valorFiltroL=100;
  AfPBH = medfilt1 (Datos(:,3), valorFiltroH);
  AfPBL = medfilt1 (Datos(:,3), valorFiltroL);
  Af= AfPBH - AfPBL;



  %Definir el tamano a usar en los graficos
  InicioTrend=20;
  %FinTrend= 1000;
  FinTrend= size(Datos(:,1))(1,1)-100;

 % close all;
  figure
  plot(TiempoA(InicioTrend:FinTrend), Af(InicioTrend:FinTrend),";Filter;");
  title(['Eje Z mag HSC  ' File]);
  xlabel('Time (s)')
  ylabel('Magnetic field')


  %Buscar cuando la curva es mayor que el umbral
  umbralpos=1.0;
  umbralneg=-1.5;
  picos=0;
  for i = InicioTrend:FinTrend
    if (Af(i-1)<umbralpos && Af(i)>umbralpos)
      tini= Tiempoms(i);
      dentropico=true;
    endif

    if (Af(i-1)>umbralpos && Af(i)<umbralpos)
      tfin= Tiempoms(i);
      if ((tfin-tini)>10)
        ++picos;
        tpico(picos)= tfin-tini;
        fpico(picos)= 1/(tpico(picos)/1000);
        dentropico=false;
      endif
    endif
  endfor

  File
  picos
  tpico;
  tpicoProm= mean(tpico);
  fpico;
  fpicoProm=1/(tpicoProm/1000);

  sampling(j,1)=tsampling;
  sampling(j,2)=fsampling;


  while (size(tpico)(1,2)>8)
    [min_value, col_index] = min(tpico);
    tpico(col_index)=[];
    fpico(col_index)=[];
  endwhile
  if (picos<8)
    tpico(8)=0;
    fpico(8)=0;
  endif
  eventos= [eventos; tpico; fpico];

  %eventos(j+6,:)= fpico;

endfor
sampling
eventos
