function varargout = Hanoi_OpenLoop_GUI(varargin)
% HANOI_OPENLOOP_GUI MATLAB code for Hanoi_OpenLoop_GUI.fig
%      HANOI_OPENLOOP_GUI, by itself, creates a new HANOI_OPENLOOP_GUI or raises the existing
%      singleton*.
%
%      H = HANOI_OPENLOOP_GUI returns the handle to a new HANOI_OPENLOOP_GUI or the handle to
%      the existing singleton*.
%
%      HANOI_OPENLOOP_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in HANOI_OPENLOOP_GUI.M with the given input arguments.
%
%      HANOI_OPENLOOP_GUI('Property','Value',...) creates a new HANOI_OPENLOOP_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Hanoi_OpenLoop_GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Hanoi_OpenLoop_GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Hanoi_OpenLoop_GUI

% Last Modified by GUIDE v2.5 15-Apr-2015 15:57:08

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Hanoi_OpenLoop_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @Hanoi_OpenLoop_GUI_OutputFcn, ...
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


% --- Executes just before Hanoi_OpenLoop_GUI is made visible.
function Hanoi_OpenLoop_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Hanoi_OpenLoop_GUI (see VARARGIN)

% Choose default command line output for Hanoi_OpenLoop_GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

global b;
global mL;
global tf;
b = 0.02;
mL = 0.1;
tf = 1;
% Set up axes
axes(handles.trajAxes);
title('Trajectory of Hanoi arm')
xlabel('Horizontal Position (cm)');
ylabel('Veritcal Position (cm)');
axis([-40 40 -40 40]);

axes(handles.angleAxes)
title('Hanoi Arm Angles')
xlabel('Time (s)')
ylabel('Angle (degrees)')
axis([0 5 -180 180]);

axes(handles.torqueAxes)
title('Total Torque on Each Arm Joint')
xlabel('Time (s)')
ylabel('Torque (N-m)')
axis([0 5 -1 1]);

% UIWAIT makes Hanoi_OpenLoop_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Hanoi_OpenLoop_GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in plotButton.
function plotButton_Callback(hObject, eventdata, handles)
% hObject    handle to plotButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% set values (ask whether they go to traj generator or arm as well)
global mL;
global b;
global tf;
mL = str2double(get(handles.LoadMassEntry,'String'));
b = str2double(get(handles.frictionEntry,'String'));
tf = str2double(get(handles.moveTimeEntry,'String'));

% run sim
[tout, ~, yout] = sim('Hanoi_OpenLoop_globals.slx', tf*5);
assignin('base','yout','yout')
p = yout(:,1:2);
theta = yout(:,3:4);
Tau = yout(:,5:6);

set(handles.plotButton, 'Enable', 'off');
% plot stuff
axes(handles.angleAxes)
plot(tout,theta*360/(2*pi));
title('Hanoi Arm Angles')
xlabel('Time (s)')
ylabel('Angle (degrees)')
legend('Joint 1 angle', 'Joint 2 angle');

axes(handles.torqueAxes)
plot(tout,Tau);
title('Total Torque on Each Arm Joint')
xlabel('Time (s)')
ylabel('Torque (N-m)')
legend('Joint 1', 'Joint 2');

axes(handles.trajAxes);
for i = [2:4:length(p(:,1)),length(p(:,1))]
    tic;
    cla
    plot(p(1:i,1), p(1:i,2));
    axis([-40 40 -40 40]);
    title('Trajectory of Hanoi arm')
    xlabel('Horizontal Position (cm)');
    ylabel('Veritcal Position (cm)');
    drawnow;
    while(toc < 0.01)
        % do nothing
    end
end
set(handles.plotButton, 'Enable', 'on');



    
function LoadMassEntry_Callback(hObject, eventdata, handles)
% hObject    handle to LoadMassEntry (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of LoadMassEntry as text
%        str2double(get(hObject,'String')) returns contents of LoadMassEntry as a double
newVal = str2double(get(hObject,'String'));
if(isnan(newVal))
     set(hObject,'String', '0.1');
end

% --- Executes during object creation, after setting all properties.
function LoadMassEntry_CreateFcn(hObject, eventdata, handles)
% hObject    handle to LoadMassEntry (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function frictionEntry_Callback(hObject, eventdata, handles)
% hObject    handle to frictionEntry (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of frictionEntry as text
%        str2double(get(hObject,'String')) returns contents of frictionEntry as a double
newVal = str2double(get(hObject,'String'));
if(isnan(newVal))
     set(hObject,'String', '0.02');
end

% --- Executes during object creation, after setting all properties.
function frictionEntry_CreateFcn(hObject, eventdata, handles)
% hObject    handle to frictionEntry (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function moveTimeEntry_Callback(hObject, eventdata, handles)
% hObject    handle to moveTimeEntry (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of moveTimeEntry as text
%        str2double(get(hObject,'String')) returns contents of moveTimeEntry as a double
newVal = str2double(get(hObject,'String'));
assignin('base', 'newVal', newVal);
if(isnan(newVal))
     set(hObject,'String', '1');
end

% --- Executes during object creation, after setting all properties.
function moveTimeEntry_CreateFcn(hObject, eventdata, handles)
% hObject    handle to moveTimeEntry (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
