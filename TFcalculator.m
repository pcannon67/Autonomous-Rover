function varargout = TFcalculator(varargin)
% TFCALCULATOR MATLAB code for TFcalculator.fig
%      TFCALCULATOR, by itself, creates a new TFCALCULATOR or raises the existing
%      singleton*.
%
%      H = TFCALCULATOR returns the handle to a new TFCALCULATOR or the handle to
%      the existing singleton*.
%
%      TFCALCULATOR('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TFCALCULATOR.M with the given input arguments.
%
%      TFCALCULATOR('Property','Value',...) creates a new TFCALCULATOR or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before TFcalculator_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to TFcalculator_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help TFcalculator

% Last Modified by GUIDE v2.5 05-Jun-2018 16:12:47

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @TFcalculator_OpeningFcn, ...
                   'gui_OutputFcn',  @TFcalculator_OutputFcn, ...
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


% --- Executes just before TFcalculator is made visible.
function TFcalculator_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to TFcalculator (see VARARGIN)

% Choose default command line output for TFcalculator
handles.output = hObject;

axes(handles.logo)
Logo = imread('logo.png');  % open file directory and show all .png files
    % set selected file as picture
imshow(Logo)                   % siplay image

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes TFcalculator wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = TFcalculator_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in LoadButton.
function LoadButton_Callback(hObject, eventdata, handles)

file = uigetfile('.txt');  % open file directory and show all .png files
data = importdata(file);
t = [0:0.01:0.74];
y = -1*data;
axes(handles.FileData)  
plot(t,y);
grid on;

axes(handles.OutputData)
x = [t]';
data = [x,y];
a0=[0; 1]; % initial value
a = lsqnonlin(@(a)myfunction(a,data),a0);
ybar =  a(1)*(1 - exp(-1*a(2)*x)); % calculate the values from the fitted model
plot(x,y,'r+',x, ybar,'b-','LineWidth',1); 
grid on;

Tra = 0.25;
K = 1.8;
zeta = 2.8;
%wn = (2.16*zeta + 0.6)/2.16  %when 0.3<= zeta <=0.7
wn = 2.5*pi/Tra;             %when 0.7<= zeta <=4
A = (wn^2)*K;
B = 2*zeta*wn;
C = wn^2;

G = tf(A,[1 B C]);

W = evalc('G'); 
set(handles.OpenLoop,'string',W);


% --- Executes on button press in StepButton.
function StepButton_Callback(hObject, eventdata, handles)
axes(handles.axes3) 
K = 1.8;
Tra = 0.25;
PO = 10;    %Percentage overshoot
Tr = 0.2;   % rise-time
zeta = 2.8;
%wn = (2.16*zeta + 0.6)/2.16  %when 0.3<= zeta <=0.7
wn = 2.5*pi/Tra;             %when 0.7<= zeta <=4
A = (wn^2)*K;
B = 2*zeta*wn;
C = wn^2;

G = tf(A,[1 B C]);

Zeta = sqrt((log(PO/100)^2)/((log(PO/100)^2)+(pi^2))); %Damping Ratio
Wn = (2.16*Zeta + 0.6)/Tr; %Natural Frequency

Po = 10*Zeta*Wn;

Ki = (((Wn^2)*Po)/(A));  % Integral Gain
Kp = (2*Zeta*Wn*(Po));               % Proportional gain

Gc = tf([Kp Ki],[1 0]);

X = ((G*Gc)/(1+(G*Gc)));
step(X)
stepdataA = stepinfo(X,'SettlingTimeThreshold',0.02);
W3 = evalc('stepdataA.RiseTime');
set(handles.RiseTime,'string',W3);
W4 = evalc('stepdataA.Overshoot');
set(handles.Overshoot,'string',W4);





% --- Executes on button press in RunButton.
function RunButton_Callback(hObject, eventdata, handles)
K = 1.8;
Tra = 0.25;
PO = 10;    %Percentage overshoot
Tr = 0.2;   % rise-time
zeta = 2.8;
%wn = (2.16*zeta + 0.6)/2.16  %when 0.3<= zeta <=0.7
wn = 2.5*pi/Tra;             %when 0.7<= zeta <=4
A = (wn^2)*K;
B = 2*zeta*wn;
C = wn^2;

G = tf(A,[1 B C]);
Zeta = sqrt((log(PO/100)^2)/((log(PO/100)^2)+(pi^2))); %Damping Ratio
Wn = (2.16*Zeta + 0.6)/Tr; %Natural Frequency
Po = 10*Zeta*Wn;

Ki = (((Wn^2)*Po)/(A));  % Integral Gain
Kp = (2*Zeta*Wn*(Po));               % Proportional gain

Gc = tf([Kp Ki],[1 0]);
W2 = evalc('Gc'); 
set(handles.CloseLoop,'string',W2);



function OpenLoop_Callback(hObject, eventdata, handles)
% hObject    handle to OpenLoop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of OpenLoop as text
%        str2double(get(hObject,'String')) returns contents of OpenLoop as a double


% --- Executes during object creation, after setting all properties.
function OpenLoop_CreateFcn(hObject, eventdata, handles)
% hObject    handle to OpenLoop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function ClosedLoop_Callback(hObject, eventdata, handles)
% hObject    handle to ClosedLoop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ClosedLoop as text
%        str2double(get(hObject,'String')) returns contents of ClosedLoop as a double


% --- Executes during object creation, after setting all properties.
function ClosedLoop_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ClosedLoop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function STR_Callback(hObject, eventdata, handles)
% hObject    handle to STR (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of STR as text
%        str2double(get(hObject,'String')) returns contents of STR as a double


% --- Executes during object creation, after setting all properties.
function STR_CreateFcn(hObject, eventdata, handles)
% hObject    handle to STR (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in ControllerChoice.
function ControllerChoice_Callback(hObject, eventdata, handles)
% hObject    handle to ControllerChoice (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns ControllerChoice contents as cell array
%        contents{get(hObject,'Value')} returns selected item from ControllerChoice


% --- Executes during object creation, after setting all properties.
function ControllerChoice_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ControllerChoice (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function ControllerGains_Callback(hObject, eventdata, handles)
% hObject    handle to ControllerGains (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ControllerGains as text
%        str2double(get(hObject,'String')) returns contents of ControllerGains as a double


% --- Executes during object creation, after setting all properties.
function ControllerGains_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ControllerGains (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
