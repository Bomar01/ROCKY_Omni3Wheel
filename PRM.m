%%%%%%%%%%% Obtención de la trayectoria de ROCKY implementando PRM %%%%%%%%%%%

%Limpieza del entorno
clear all; close all; clc;
%Lectura de la imagen original
img = imread('mapa_pers3.jpg');
figure()
imshow(img)
pause(2)
%Dimensiones de la imagen deseada [px]
tamx=3000;
tamy=1600;
%Puntos de correspondencia
pts1 = [758, 179; 3018, 104; 582, 1780; 3325, 1639];
pts2 = [0, 0; tamx, 0; 0, tamy; tamx, tamy];
%Dimensiones de la imagen original
[alto, ancho, canales] = size(img);
disp("Ancho: " + ancho);
disp("Alto: " + alto);
[rows, cols, ch] = size(img);
%Transformacipon proyectiva
M = fitgeotrans(pts1, pts2, 'projective');
dst = imwarp(img, M, 'OutputView', imref2d([tamy, tamx]));
im=dst;
figure()
imshow(im)
pause(2)

%Binarización
I2=binarizarMapa(im);
figure();
imshow(I2)
pause(2)
%Erosión y dilatación
If=I2;
se=strel('disk',9); 
If=imerode(If,se);
figure();
imshow(If)
pause(2)
se=strel('disk',150); 
If=imdilate(If,se);
figure();
imshow(If)
pause(2)

%Parámetros de conversion de píxeles a metros
resx=3.06/tamx;
resy=2.04/tamy;
%Punto incial y final [m]
xi=0.34;
yi=0.34;
xd=2.8;
yd=1.36;
%Conversion [px]
xim=xi/resx
yim=yi/resy
xdm=xd/resx
ydm=yd/resy

%Punto inicial y meta [px]
startPoint=[xim yim];
goalPoint=[xdm ydm];

% Planificación de la ruta usando el mapa binarizado
figure()
map=robotics.BinaryOccupancyGrid(If, 1);
planner=robotics.PRM(map);
planner.NumNodes=150;
planner. ConnectionDistance=600;
    waypoints=findpath(planner,startPoint,goalPoint);
    show(planner)
    pause(2)
%Conversion de los puntos de la ruta(px a m)
puntosx=((waypoints(:,1)-xim))*resx
puntosx=puntosx+0.1
puntosy=(waypoints(:,2)-yim)*resy
