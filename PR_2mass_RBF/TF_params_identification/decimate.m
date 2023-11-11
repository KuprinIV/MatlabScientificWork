function [sys,x0,str,ts] = decimate(t,x,u,flag,tq,decf,uy)
% ��� - S-�������
% ���� ������������ ��� ������ �� ���������. ���� ��� �������������� ����������, ��
% ������ ��������� ���������: function [sys,x0,str,ts] = ���(t,x,u,flag);
% ��� "���" - ��� ������� ����������� ����� S-Function � ������ ���������. ���
% ����������� ������ �������������� ����� �-�����, � ������� ��� ������� �����������.
% ��������� ��������� �������� ������������� � ��������� �� ��������. ����� ���� ��������
% �������� ����� ����� flag, �� ���������� ��� "���������" � � S-Function � ������ ���������
% ����� ������� �����. ��� ����� ������� ����� �����������. ����������  ��������
% �������������� ����� �����
% ����� ����:
%		0 - ������������� ������� - ��� ������ �� �������
%		2 - ���������� ���������� ���������� � ��������� �������, �.� �����������
%			 ���������� �� ������� ������ ������� ���������� ���������.
%			 �������� ���������� - ������� sys - ������������ ������� x
%		3 - ������������ �������� �������� ������� - ����� ������ sys
% � ������ ������ �� ������� �� ������ ���������� ����������� �������.
% ������, �� ��������� �������� ���� ������� ������������ ����������
% ��������� �������������.
% ����� � ������ ���������� ����� �������!!!
switch flag
% Initialization
  case 0
    [sys,x0,str,ts]=mdlInitializeSizes(tq,decf);
% Form outputs
  case 3
   sys=mdlOutputs(t,x,u,decf,uy);
   
% Unhandled flags
  case { 1, 2, 4, 9 }
    sys=[];
% Unexpected flags 
  otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end

% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the
% S-function.
% ���������� ������, ��������� �������� � ��� ����������� ������  ��� �������
function [sys,x0,str,ts]=mdlInitializeSizes(tq,decf)
%"Global" word need in that part of program (in this function)
% where global variable is used

% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%�����������!!!
sizes = simsizes; % �������� �� ���������, ������� ���������� ���������� ���
                  % ���������� ������ S-�������

sizes.NumContStates  = 0; % ����� ����������� ���������� ���������
sizes.NumDiscStates  = 0; % ����� ���������� ���������� ���������
sizes.NumOutputs     = 2; % ����� �������
sizes.NumInputs      = 2; % ����� ������
sizes.DirFeedthrough = 1; % ???
sizes.NumSampleTimes = 1; % �� ������� ���� ���� ��� ����������� ���������
% at least one sample time is needed

sys = simsizes(sizes);
% initialize the initial conditions
% ������� ��������� ������� - ��� ����������� ������� ���������
x0  = [];
% str is always an empty matrix
% str - ������ ������ �������
str = [];
% initialize the array of sample times
% ������������� ������� ������ �����������
% ������ ������� - ���� �����������
ts  = [tq*decf 0];

function sys=mdlOutputs(t,x,u,decf,uy);
sys(1) = u(1);
sys(2) = u(2)./uy;
