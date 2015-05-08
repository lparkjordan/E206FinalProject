function varargout = Hanoi_SlidingControl_GUI(varargin)
% HANOI_SLIDINGCONTROL_GUI MATLAB code for Hanoi_SlidingControl_GUI.fig
%      HANOI_SLIDINGCONTROL_GUI, by itself, creates a new HANOI_SLIDINGCONTROL_GUI or raises the existing
%      singleton*.
%
%      H = HANOI_SLIDINGCONTROL_GUI returns the handle to a new HANOI_SLIDINGCONTROL_GUI or the handle to
%      the existing singleton*.
%
%      HANOI_SLIDINGCONTROL_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in HANOI_SLIDINGCONTROL_GUI.M with the given input arguments.
%
%      HANOI_SLIDINGCONTROL_GUI('Property','Value',...) creates a new HANOI_SLIDINGCONTROL_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Hanoi_SlidingControl_GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Hanoi_SlidingControl_GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Hanoi_SlidingControl_GUI

% Last Modified by GUIDE v2.5 07-May-2015 20:45:18

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Hanoi_SlidingControl_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @Hanoi_SlidingControl_GUI_OutputFcn, ...
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


% --- Executes just before Hanoi_SlidingControl_GUI is made visible.
function Hanoi_SlidingControl_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Hanoi_SlidingControl_GUI (see VARARGIN)

% Choose default command line output for Hanoi_SlidingControl_GUI
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
% axis([-40 40 -40 40]);
axis([-5 35 -15 25]);

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

% UIWAIT makes Hanoi_SlidingControl_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Hanoi_SlidingControl_GUI_OutputFcn(hObject, eventdata, handles) 
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
global epsilon;
% epsilon = 1;
epsilon = str2double(get(handles.epsilonEntry,'String'));

mL = str2double(get(handles.LoadMassEntry,'String'));
b = str2double(get(handles.frictionEntry,'String'));
tf = str2double(get(handles.moveTimeEntry,'String'));
U = str2double(get(handles.SC_GainEntry,'String'));
assignin('base','U',U);
G = [zeros(2,2);eye(2)];
assignin('base','G',G);
P = eye(4);
assignin('base','P',P);

Kp = eye(2)*316;
Kd = eye(2)*14;
assignin('base','Kp',Kp);
assignin('base','Kd',Kd);

% run sim
[tout, ~, yout] = sim('FeedbackLinearizedArm.slx', tf*5);
assignin('base','yout','yout')
p = yout(:,1:2);
theta = yout(:,3:4);
Tau = yout(:,5:6);
pgoal = yout(:,7:8);

error = norm(p-pgoal);

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
cla;
a = 20;
n = 1;
plot(p(1,1),p(1,2),'o-');
% plot(pgoal(1,1),pgoal(1,2),'o-','Color', 'Green');
link1 = line([0 a*cos(theta(n,1))], [0 a*sin(theta(n,1))],'LineWidth',2, 'Color','Red');
link2 = line([a*cos(theta(n,1)) a*cos(theta(n,1)) + a*cos(theta(n,1)+theta(n,2))], ...
         [a*sin(theta(n,1)) a*sin(theta(n,1)) + a*sin(theta(n,1)+theta(n,2))],'LineWidth',2,'Color','Red');
xlim(gca,[-40 40])
ylim(gca,[-40 40])
title('Robot Manipulator')
xlabel('X position (cm)')
ylabel('Y position (cm)')
drawnow        

skip = 1;

tic
t = toc;
for i = [2:skip:length(p(:,1)),length(p(:,1))]
%     tic;
    t = t+0.01;
    delete(link1)
    delete(link2)
%     cla(handles.axes_SimPlot);    
    hold all;
    plot(p(i-skip:i,1),p(i-skip:i,2),'o-', 'Color', 'Blue', 'MarkerSize',3);
    link1 = line([0 a*cos(theta(i,1))], [0 a*sin(theta(i,1))],'LineWidth',2, 'Color','Red');
    link2 = line([a*cos(theta(i,1)) a*cos(theta(i,1)) + a*cos(theta(i,1)+theta(i,2))], ...
                 [a*sin(theta(i,1)) a*sin(theta(i,1)) + a*sin(theta(i,1)+theta(i,2))],'LineWidth',2,'Color','Red');
    
    
%     plot(p(1:i,1), p(1:i,2));
    axis([-5 35 -15 25]);
    title('Trajectory of Hanoi arm')
    xlabel('Horizontal Position (cm)');
    ylabel('Veritcal Position (cm)');
    drawnow;
    
    while (toc < t)
        %do nothing
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



function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double


% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function epsilonEntry_Callback(hObject, eventdata, handles)
% hObject    handle to epsilonEntry (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of epsilonEntry as text
%        str2double(get(hObject,'String')) returns contents of epsilonEntry as a double
newVal = str2double(get(hObject,'String'));
if(isnan(newVal))
     set(hObject,'String', '0.1');
end 

% --- Executes during object creation, after setting all properties.
function epsilonEntry_CreateFcn(hObject, eventdata, handles)
% hObject    handle to epsilonEntry (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function SC_GainEntry_Callback(hObject, eventdata, handles)
% hObject    handle to SC_GainEntry (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of SC_GainEntry as text
%        str2double(get(hObject,'String')) returns contents of SC_GainEntry as a double
newVal = str2double(get(hObject,'String'));
if(isnan(newVal))
     set(hObject,'String', '0.1');
end

% --- Executes during object creation, after setting all properties.
function SC_GainEntry_CreateFcn(hObject, eventdata, handles)
% hObject    handle to SC_GainEntry (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
