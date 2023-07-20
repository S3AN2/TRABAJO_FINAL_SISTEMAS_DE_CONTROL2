function varargout = TRABAJO2(varargin)

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @TRABAJO2_OpeningFcn, ...
                   'gui_OutputFcn',  @TRABAJO2_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before TRABAJO2 is made visible.
function TRABAJO2_OpeningFcn(hObject, eventdata, handles, varargin)


axes(handles.axes4);
path = '../assets/FIEE.jpg';
img = imread(path);
imshow(img);
axis off;
axes(handles.axes3);
path = '../assets/UNMSM.jpg';
img = imread(path);
imshow(img);
axis off;
axes(handles.axes7);
path = '../assets/PI.jpg';
img = imread(path);
imshow(img);
axis off;

find_system('Name','../SIMULINK/control2');
open_system('../SIMULINK/control2');
handles.output = hObject;
guidata(hObject, handles);


% --- Outputs from this function are returned to the command line.
function varargout = TRABAJO2_OutputFcn(hObject, eventdata, handles) 

varargout{1} = handles.output;


% --- Executes on button press in ACEPTAR.
function ACEPTAR_Callback(hObject, eventdata, handles)
global espacio
global funcion
global deseado
global PID
global loca
global fttt
global G7
global G11
global controo
global contro
global G222
global pr
param_K1=get(handles.K1,'string');
param_B1=get(handles.B1,'string');
param_M1=get(handles.M1,'string');
param_T1=get(handles.T1,'string');
param_MP=get(handles.MP,'string');
param_TSS=get(handles.TSS,'string');
Mp1=eval(param_MP);
Mp=Mp1/100;
Tss=eval(param_TSS);
num=1;
den20=eval(param_T1);
den1=eval(param_K1);
den2=eval(param_B1);
den3=eval(param_M1);
den4=1/den3;%G
den5=den2/den3;%G2
den6=den1/den3;%G3
den=[den3 den2 den1];
denw=den/den3;
numw=num/den3;
funcion_de_transferencia=tf(numw,denw);
fttt=funcion_de_transferencia;
%
k = numw;
Ps = denw;
%
Zeta = sqrt((log(Mp))^2/(pi^2+(log(Mp))^2)); %factor de amortiguamiento
Wn = 4/(Zeta*Tss);%frecuencia natural del sistema
%Ecuacion deseada
Gd = tf([Wn^2],[1 2*Wn*Zeta Wn^2]);

%Calcular los polos del sistema deseado
Polos_deseados = pole(Gd) ;%Raices complejas 
Polo_3 = 10*real(Polos_deseados);

%calculo de los parametros del PID
%polinomio deseado
p1 = conv([1 -real(Polos_deseados(1))],[1 -real(Polos_deseados(1))]) + [0 0 imag(Polos_deseados(1))^2];
%ecuacion deseada
Ecuacion_deseada= conv([1 -Polo_3(1)],p1);
Ecuacion_deseada_del_PID=Ecuacion_deseada;
%valores del PID
Kc = (Ecuacion_deseada(3)-Ps(3))/k;
Ti = (k*Kc)/Ecuacion_deseada(4);
Td = (Ecuacion_deseada(2)-Ps(2))/(k*Kc);
N=char('KC : ','Ti : ','Td : ');
R=[Kc;Ti;Td];
S=num2str(R);
PARAMETROS_DEL_CONTROLADOR_PID=[N,S];
%
poolos=[(Polos_deseados(1)) (Polos_deseados(2))];
%
[A,B,C,D]=tf2ss(num,den);
funcion_de_transferencia=tf(num,den);
mA=A;
mB=B;
mC=C;
mD=D;
%
%
%
A0=mA;
B0=mB;
C0=mC;
D0=mD;
[nu,de]=ss2tf(A0,B0,C0,D0);
GG=tf(nu,de);
%controlabilidad
Coo=ctrb(A0,B0);
dime=size(Coo);
Dimension=dime(1);
rangoC=rank(Coo);
Rango_de_la_controlabilidad=rangoC;
%
N1=char('Controlabilidad : ');
N3=char('Dimension: ');
N2=char('Rango: ');
S2=mat2str(Coo);
S4=mat2str(Dimension);
S3=mat2str(rangoC);
hhh=[N1,S2];
HHH=[N2,S3];
kkk=[N3,S4];
Controlabilidad_del_sistema=strvcat(hhh,kkk,HHH);
%
Oo=obsv(A0,C0);
rangoO=rank(Oo);
K=place(A0, B0, poolos);
AA=A0-B0*K;
[num0,den0]=ss2tf(AA,B0,C0,D0);
G11=tf(num0,den0);
%
%
precomp=(((-1)*C0*((AA)^(-1))*B0))^(-1);
G222=precomp*G11;
Ganancia_de_pre_compensacion=precomp;

%
%
Kpimc=Kc;
%
s=tf('s');
G1=funcion_de_transferencia;
sys1=parallel(Kpimc,Kpimc*1/(Ti*s));
GPID=parallel(sys1,Kpimc*s*Td);
G7=feedback(GPID*G1,1);


%
%
set_param('control2/G','Gain',num2str(den4));
set_param('control2/G2','Gain',num2str(den5));
set_param('control2/G3','Gain',num2str(den6));
%
set_param('control2/tf','Numerator',mat2str([1]),'Denominator',mat2str(den));
%
%
set_param('control2/tf1','Numerator',mat2str([1]),'Denominator',mat2str(den));
%
set_param('control2/stsp','A',mat2str(mA),'B',mat2str(mB),'C',mat2str(mC),'D',mat2str(mD));
%
set_param('control2','StopTime',num2str(den20));
%
%
%PID
set_param('control2/PID','P',num2str(Kc));
set_param('control2/PID','I',num2str(Kc/Ti));
set_param('control2/PID','D',num2str(Kc*Td));
%
%
%CONTROLADOR POR LOCALIZACION DE POLOS
set_param('control2/G4','Gain',mat2str(B0));
set_param('control2/G5','Gain',mat2str(A0));
set_param('control2/G6','Gain',mat2str(K));
set_param('control2/G7','Gain',mat2str(C0));
%
%CONTROLADOR POR LOCALIZACION DE POLOS
set_param('control2/G13','Gain',mat2str(B0));
set_param('control2/G9','Gain',mat2str(A0));
set_param('control2/G10','Gain',mat2str(K));
set_param('control2/G11','Gain',mat2str(C0));
%PRE-COMPENSADOR
set_param('control2/G8','Gain',num2str(precomp));
%
%

Ecuacion_deseada_del_control_por_localizacion=p1;
funcion=evalc('funcion_de_transferencia');
PID=evalc('PARAMETROS_DEL_CONTROLADOR_PID');
deseado=evalc('Ecuacion_deseada_del_PID');
loca=evalc('K');
contro=evalc('Controlabilidad_del_sistema');
controo=evalc('Ecuacion_deseada_del_control_por_localizacion');
pr=evalc('Ganancia_de_pre_compensacion');



% --- Executes on button press in RETROCEDER.
function RETROCEDER_Callback(hObject, eventdata, handles)
close(TRABAJO2)
TRABAJO1



function K1_Callback(hObject, eventdata, handles)

function K1_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function B1_Callback(hObject, eventdata, handles)

function B1_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function M1_Callback(hObject, eventdata, handles)

function M1_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function MP_Callback(hObject, eventdata, handles)

function MP_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function TSS_Callback(hObject, eventdata, handles)

function TSS_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function listbox1_Callback(hObject, eventdata, handles)
global deseado
global loca
global funcion
global PID
global contro
global controo
global pr
inf=get(hObject,'Value');
gos=get(hObject,'String');
switch inf
 case 2
 set(handles.text122,'string',funcion)
 case 3
 set(handles.text122,'string',deseado)
  case 4
 set(handles.text122,'string',controo)
  case 5
 set(handles.text122,'string',contro)
  case 6
 set(handles.text122,'string',PID)
  case 7
 set(handles.text122,'string',loca)
    case 8 
 set(handles.text122,'string',pr)
end
guidata(hObject,handles);

function listbox1_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


%lista 2
function listbox2_Callback(hObject, eventdata, handles)

global fttt
global G7
global G11
global G222
inf=get(hObject,'Value');
gos=get(hObject,'String');
switch inf
 case 2
set(handles.axes5); % Establece los ejes de graficación
axes(handles.axes5);
step(fttt);grid on;
title('Función de transferencia')
 case 3
set(handles.axes5); % Establece los ejes de graficación
axes(handles.axes5);
step(G7);grid on;
title('CONTROLADOR PID')
  case 4
set(handles.axes5); % Establece los ejes de graficación
axes(handles.axes5);
step(G11);grid on;
title('CONTROLADOR POR LOCALIZACION DE POLOS')
  case 5
set(handles.axes5); % Establece los ejes de graficación
axes(handles.axes5);
step(G11);
hold on
step(G7);grid on;
hold off
title('COMPARACIÓN AMBAS GRÁFICAS')

 case 6
set(handles.axes5); % Establece los ejes de graficación
axes(handles.axes5);
step(G222);grid on;
title(' GANANCIA PRE-COMPENSACIÓN')
 case 7
set(handles.axes5); % Establece los ejes de graficación
axes(handles.axes5);
step(G222);grid on;
hold on
step(G7);grid on;
hold off
title('GANANCIA PRE-COMPENSACIÓN Y PID')
end
guidata(hObject,handles);

function listbox2_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pushbutton3_Callback(hObject, eventdata, handles)
set_param(gcs,'SimulationCommand','Start');
open_system('control2/Scope3');




function T1_Callback(hObject, eventdata, handles)



% --- Executes during object creation, after setting all properties.
function T1_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
set_param(gcs,'SimulationCommand','Start');
open_system('control2/Scope1');


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
set_param(gcs,'SimulationCommand','Start');
open_system('control2/Scope2');


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
set_param(gcs,'SimulationCommand','Start');
open_system('control2/Scope4');
