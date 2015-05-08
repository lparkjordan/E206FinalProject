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

% Last Modified by GUIDE v2.5 07-May-2015 23:36:27

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
global Iz;
b = 0.02;
mL = 0.1;
tf = 1;
Iz = 0.01;

% Set up axes
axes(handles.errorAxes)
cla;
% plot(tout,error);
title('Hanoi Position Errors')
xlabel('Time (s)')
ylabel('Error (meters)')


axes(handles.trajAxes);
title('Trajectory of Hanoi arm')
xlabel('Horizontal Position (cm)');
ylabel('Veritcal Position (cm)');
% axis([-40 40 -40 40]);
axis([-5 35 -15 25]);

axes(handles.angleAxes)
title('Eta')
xlabel('Time (s)')
ylabel('Eta')
% axis([0 5 -180 180]);

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

assignin('base','stop',0);

% set values (ask whether they go to traj generator or arm as well)
global mL;
global b;
global tf;
global Iz;
global epsilon;
% epsilon = 1;
epsilon = str2double(get(handles.epsilonEntry,'String'));

mL = str2double(get(handles.LoadMassEntry,'String'));
b = str2double(get(handles.frictionEntry,'String'));
tf = str2double(get(handles.moveTimeEntry,'String'));
Iz = str2double(get(handles.linkInertiaEntry,'String'));
U = str2double(get(handles.SC_GainEntry,'String'));
assignin('base','U',U);
G = [zeros(2,2);eye(2)];
assignin('base','G',G);
P = eye(4);
assignin('base','P',P);

% Kp = eye(2)*447;
% Kd = eye(2)*43.5;
Kp = eye(2)*1414.2;
Kd = eye(2)*61.9;
% Kp = eye(2)*100;
% Kd = eye(2)*101;
assignin('base','Kp',Kp);
assignin('base','Kd',Kd);

% run sim
[tout, ~, yout] = sim('FeedbackLinearizedArm.slx', tf*5);
assignin('base','yout','yout')
p = yout(:,1:2);
theta = yout(:,3:4);
Tau = yout(:,5:6);
pgoal = yout(:,7:8);
eta = yout(:,9:10);
etaNorm = sqrt(eta(:,1).^2 + eta(:,2).^2);
maxEta = max(etaNorm);
assignin('base','eta',eta);
assignin('base','maxEta',maxEta);
% assignin('base','p',p);
% assignin('base','pgoal',pgoal);
% error = norm(p-pgoal);
pdiff = p - pgoal;
error = 10*sqrt(pdiff(:,1).^2 + pdiff(:,2).^2); % in mm
% assignin('base','error',error);
errorsquare = error.^2;
dt = 0.01;
sumErrorSquare = sum(errorsquare*dt);

sumTau1 = sum(Tau(:,1).^2)*dt;
sumTau2 = sum(Tau(:,2).^2)*dt;

moves24 = find((tf<=tout & tout<2*tf) | (3*tf<=tout & tout<4*tf));
% errors24 = error(moves24);
errorsX24 = 10*abs(pdiff(moves24,1)); % in mm
maxdeviation24 = max(errorsX24);

set(handles.minUDisplay,'String', num2str(maxEta,3));

set(handles.completionTimeDisplay,'String', num2str(tf*5));
set(handles.ISerrorDisplay,'String', num2str(sumErrorSquare));
set(handles.IStorque1Display,'String', num2str(sumTau1));
set(handles.IStorque2Display,'String', num2str(sumTau2));
set(handles.deviationDisplay,'String', num2str(maxdeviation24));
set(handles.plotButton, 'Enable', 'off');
% plot stuff
axes(handles.errorAxes)
cla;
hold all
plot(tout,pdiff(:,1).*10);
plot(tout,pdiff(:,2).*10);
title('Hanoi Position Errors')
xlabel('Time (s)')
ylabel('Error (mm)')
legend('X errors','Y errors');

% axes(handles.angleAxes)
% plot(tout,theta*360/(2*pi));
% title('Hanoi Arm Angles')
% xlabel('Time (s)')
% ylabel('Angle (degrees)')
% legend('Joint 1 angle', 'Joint 2 angle');

axes(handles.angleAxes)
cla;
hold all;
plot(tout,eta(:,1));
plot(tout,eta(:,2));
plot(tout,etaNorm,'LineWidth', 2)
title('Eta')
xlabel('Time (s)')
ylabel('Eta')
legend('Eta1', 'Eta2','Eta Norm');
axis auto
ylims = ylim;
ylim([ylims(1) ylims(2)*1.3]);

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
    plot(pgoal(i-skip:i,1),pgoal(i-skip:i,2),'o-', 'Color', 'Green', 'MarkerSize',1);
    link1 = line([0 a*cos(theta(i,1))], [0 a*sin(theta(i,1))],'LineWidth',2, 'Color','Red');
    link2 = line([a*cos(theta(i,1)) a*cos(theta(i,1)) + a*cos(theta(i,1)+theta(i,2))], ...
                 [a*sin(theta(i,1)) a*sin(theta(i,1)) + a*sin(theta(i,1)+theta(i,2))],'LineWidth',2,'Color','Red');
    
    
%     plot(p(1:i,1), p(1:i,2));
    axis([-5 35 -15 25]);
    title('Trajectory of Hanoi arm')
    xlabel('Horizontal Position (cm)');
    ylabel('Veritcal Position (cm)');
    drawnow;
    
    stop = evalin('base','stop');
    if (stop == 1)
        set(handles.plotButton, 'Enable', 'on');
        return;
    end    
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
     set(hObject,'String', '10');
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



function edit8_Callback(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit8 as text
%        str2double(get(hObject,'String')) returns contents of edit8 as a double


% --- Executes during object creation, after setting all properties.
function edit8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function completionTimeDisplay_Callback(hObject, eventdata, handles)
% hObject    handle to completionTimeDisplay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of completionTimeDisplay as text
%        str2double(get(hObject,'String')) returns contents of completionTimeDisplay as a double


% --- Executes during object creation, after setting all properties.
function completionTimeDisplay_CreateFcn(hObject, eventdata, handles)
% hObject    handle to completionTimeDisplay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ISerrorDisplay_Callback(hObject, eventdata, handles)
% hObject    handle to ISerrorDisplay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ISerrorDisplay as text
%        str2double(get(hObject,'String')) returns contents of ISerrorDisplay as a double


% --- Executes during object creation, after setting all properties.
function ISerrorDisplay_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ISerrorDisplay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function deviationDisplay_Callback(hObject, eventdata, handles)
% hObject    handle to deviationDisplay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of deviationDisplay as text
%        str2double(get(hObject,'String')) returns contents of deviationDisplay as a double


% --- Executes during object creation, after setting all properties.
function deviationDisplay_CreateFcn(hObject, eventdata, handles)
% hObject    handle to deviationDisplay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function IStorque1Display_Callback(hObject, eventdata, handles)
% hObject    handle to IStorque1Display (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of IStorque1Display as text
%        str2double(get(hObject,'String')) returns contents of IStorque1Display as a double


% --- Executes during object creation, after setting all properties.
function IStorque1Display_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IStorque1Display (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function IStorque2Display_Callback(hObject, eventdata, handles)
% hObject    handle to IStorque2Display (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of IStorque2Display as text
%        str2double(get(hObject,'String')) returns contents of IStorque2Display as a double


% --- Executes during object creation, after setting all properties.
function IStorque2Display_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IStorque2Display (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in stopButton.
function stopButton_Callback(hObject, eventdata, handles)
% hObject    handle to stopButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
assignin('base','stop',1);



function linkInertiaEntry_Callback(hObject, eventdata, handles)
% hObject    handle to linkInertiaEntry (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of linkInertiaEntry as text
%        str2double(get(hObject,'String')) returns contents of linkInertiaEntry as a double
newVal = str2double(get(hObject,'String'));
if(isnan(newVal))
     set(hObject,'String', '0.01');
end

% --- Executes during object creation, after setting all properties.
function linkInertiaEntry_CreateFcn(hObject, eventdata, handles)
% hObject    handle to linkInertiaEntry (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in restoreDefaultsButton.
function restoreDefaultsButton_Callback(hObject, eventdata, handles)
% hObject    handle to restoreDefaultsButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.LoadMassEntry,'String', '0.1');
set(handles.frictionEntry,'String', '0.02');
set(handles.linkInertiaEntry,'String', '0.01');
set(handles.moveTimeEntry,'String', '1');
set(handles.SC_GainEntry,'String', '10');
set(handles.epsilonEntry,'String', '0.10');



function minUDisplay_Callback(hObject, eventdata, handles)
% hObject    handle to minUDisplay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of minUDisplay as text
%        str2double(get(hObject,'String')) returns contents of minUDisplay as a double


% --- Executes during object creation, after setting all properties.
function minUDisplay_CreateFcn(hObject, eventdata, handles)
% hObject    handle to minUDisplay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
