clear all
clc
% pregunta 1
display('PREGUNTA NUMERO 1')
A = [2 0 0; 0 2 0 ; 0 3 1 ];
B= [0 1; 1 0 ; 0 1 ];
C=[1 0 0; 0 1 0];
D=[0];
Mx=ss(A,B,C,D)
mctr=ctrb(Mx.A,Mx.B)%el comando ctrb es para hallar la controlabilidad de la matriz
MX=mctr*mctr'%acondicionar la matriz para obtener una cuadrada
Mxdet=det(MX)%si la determinante es diferente de 0 entonces es linealmente independiente
M2=rank(MX)%rango de la matriz controlable 
[fila,columna]=size(MX)
if (fila==(M2))
display('es controlable')
else 
display('no es controlable')
end
%
%
display('PREGUNTA NUMERO 1.2')
%pregunta 1.2

A = [1 2 0;0 -1 0;2 2 0 ];
B= [-4 ; 0 ; -5];
C=[1 0 0; 0 1 0];
D=[0];
Mx=ss(A,B,C,D)
mctr=ctrb(Mx.A,Mx.B)%el comando ctrb es para hallar la controlabilidad de la matriz
MX=mctr*mctr'%acondicionar la matriz para obtener una cuadrada
Mxdet=det(MX)%si la determinante es diferente de 0 entonces es linealmente independiente
M2=rank(MX)%rango de la matriz controlable 
[fila,columna]=size(MX)
if (fila==(M2))
display('es controlable')
else 
display('no es controlable')
end


display('PREGUNTA NUMERO 2')
%pregunta 2
A = [0 1 0; 0 0 1; -6 -11 -6 ];
B= [0 ;0 ;1];
C=[4 5 1];
D=[0];
Mx=ss(A,B,C,D)
M2=obsv(Mx.A,Mx.C)
M2rango=rank(M2)
[fila,columna]=size(M2)
if (fila==rank(M2))
display('es observable')
else 
display('no es observable')
end
%
%
display('PREGUNTA NUMERO 2.1')
%pregunta 2.1

A = [1 2 0;0 -1 0;2 2 0 ];
B= [-4 ; 0 ; -7];
C=[1 0 0];
D=[0];
Mx=ss(A,B,C,D)
M2=obsv(Mx.A,Mx.C)
M2rango=rank(M2)
[fila,columna]=size(M2)
if (fila==M2rango)
display('es observable')
else 
display('no es observable')
end
% pregunta 3
display('PREGUNTA NUMERO 3 CONTROLABILIDAD')
%CONTROLABILIDAD
num=[1 7 10];
den=[1 8 19 122];
[A,B,C,D]=tf2ss(num,den)
Mx=ss(A,B,C,D)
mctr=ctrb(Mx.A,Mx.B)%el comando ctrb es para hallar la controlabilidad de la matriz
MX=mctr*mctr'%acondicionar la matriz para obtener una cuadrada
Mxdet=det(mctr)%si la determinante es diferente de 0 entonces es linealmente independiente
M5=rank(mctr)%rango de la matriz controlable 
[fila,columna]=size(MX)
if M5== fila
display('es controlable')
else 
display('no es controlable')
end
display('PREGUNTA NUMERO 3 OBSERVABILIDAD')
% OBSERVABILIDAD
den=[1 8 19 122];
[A,B,C,D]=tf2ss(num,den)
Mx=ss(A,B,C,D)
mctr=obsv(Mx.A,Mx.C)%el comando ctrb es para hallar la controlabilidad de la matriz
MX=mctr*mctr'%acondicionar la matriz para obtener una cuadrada
Mxdet=det(mctr)%si la determinante es diferente de 0 entonces es linealmente independiente
M5=rank(mctr)%rango de la matriz controlable 
[fila,columna]=size(MX)
if M5== fila
display('es observable')
else 
display('no es observable')
end
%%
%PREGUNTA D
L=1;
R=100;
C=1;
s=tf('s');
G=(L*s)/(R/(C*s*R+1)+L*s)
[fnum,fden]=tfdata(G);%dato de la funcion de transferencia
celldisp (fnum);%para leer fnum dado por tfdata
fnum= fnum{1};%para obtener el valor almacenado
celldisp(fden);%para leer fden dado por tfdata
fden{1};%para obtener el valor almacenado
fden=ans;%guardar el valor 
[A,B,C,D]=tf2ss(fnum,fden)%comando tf2ss para pasar de ft a ss
Mx=ss(A,B,C,D)
mctr=ctrb(Mx.A,Mx.B)%el comando ctrb es para hallar la controlabilidad de la matriz
MX=mctr*mctr'%acondicionar la matriz para obtener una cuadrada
Mxdet=det(MX)%si la determinante es diferente de 0 entonces es linealmente independiente
M2=rank(MX)%rango de la matriz controlable 
[fila,columna]=size(MX)
if (fila==(M2))
display('es controlable')
else 
display('no es controlable')
end