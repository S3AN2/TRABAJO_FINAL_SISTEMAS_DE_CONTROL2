function varargout = TRABAJO1(varargin)

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @TRABAJO1_OpeningFcn, ...
                   'gui_OutputFcn',  @TRABAJO1_OutputFcn, ...
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
function TRABAJO1_OpeningFcn(hObject, eventdata, handles, varargin)

axes(handles.axes2);
path = '../assets/FIEE.jpg';
img = imread(path);
imshow(img);
axis off;
axes(handles.axes1);
path = '../assets/UNMSM.jpg';
img = imread(path);
imshow(img);
axis off;
axes(handles.axes5);
path = '../assets/masa.jpg';
img = imread(path);
imshow(img);
axis off;

find_system('Name','../SIMULINK/control2');
open_system('../SIMULINK/control2');
handles.output = hObject;
guidata(hObject, handles);


function varargout = TRABAJO1_OutputFcn(hObject, eventdata, handles) 

varargout{1} = handles.output;



function K_Callback(hObject, eventdata, handles)
% 


% --- Executes during object creation, after setting all properties.
function K_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function B_Callback(hObject, eventdata, handles)

function B_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function M_Callback(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function M_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function pushbutton1_Callback(hObject, eventdata, handles)
global espacio
global funcion
global controlable
global obser
global diag
param_K=get(handles.K,'string');
param_B=get(handles.B,'string');
param_M=get(handles.M,'string');
param_T=get(handles.T,'string');
num=1;
den0=eval(param_T);
den1=eval(param_K);
den2=eval(param_B);
den3=eval(param_M);
den4=1/den3;%G
den5=den2/den3;%G2
den6=den1/den3;%G3
den=[den3 den2 den1];
funcion_de_transferencia=tf(num,den)
[A,B,C,D]=tf2ss(num,den)

mA=A;
mB=B;
mC=C;
mD=D;
forma_canonica_controlable=ss(A,B,C,D);
forma_canonica_diagonal= canon(forma_canonica_controlable,'modal');
forma_canonica_observable=canon(forma_canonica_controlable,'companion');
%
%

%
funcion=evalc('funcion_de_transferencia')
espacio=evalc('forma_canonica_controlable')
diag=evalc('forma_canonica_diagonal')
obser=evalc('forma_canonica_observable')
%
set_param('control2/G','Gain',num2str(den4));
set_param('control2/G2','Gain',num2str(den5));
set_param('control2/G3','Gain',num2str(den6));
%
set_param('control2/tf','Numerator',mat2str([1]),'Denominator',mat2str(den));
%
set_param('control2/stsp','A',mat2str(mA),'B',mat2str(mB),'C',mat2str(mC),'D',mat2str(mD));
%
set_param('control2','StopTime',num2str(den0));
%
% set(handles.text11,'string',funcion)
% set(handles.text12,'string',espacio)
set(handles.axes3); % Establece los ejes de graficación
axes(handles.axes3);
step(funcion_de_transferencia);grid on;
title('Función de transferencia')
set(handles.axes4); % Establece los ejes de graficación
axes(handles.axes4);
step(forma_canonica_controlable);grid on;
title('Gráfica de Espacio Estados')
set_param(gcs,'SimulationCommand','Start');
open_system('control2/Scope');

function T_Callback(hObject, eventdata, handles)
% hObject    handle to T (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of T as text
%        str2double(get(hObject,'String')) returns contents of T as a double


% --- Executes during object creation, after setting all properties.
function T_CreateFcn(hObject, eventdata, handles)
% hObject    handle to T (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in listbox1.
function listbox1_Callback(hObject, eventdata, handles)
global espacio
global diag
global funcion
global obser
inf=get(hObject,'Value');
gos=get(hObject,'String');
switch inf
 case 2
 set(handles.text12,'string',funcion)
 case 3
 set(handles.text12,'string',espacio)
  case 4
 set(handles.text12,'string',obser)
  case 5
 set(handles.text12,'string',diag)
end
guidata(hObject,handles);


function listbox1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
close(TRABAJO1)
TRABAJO2
