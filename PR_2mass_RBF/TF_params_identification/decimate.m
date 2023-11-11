function [sys,x0,str,ts] = decimate(t,x,u,flag,tq,decf,uy)
% Это - S-функция
% Есть определенный ряд правил ее написания. Если нет дополнительных параметров, то
% формат заголовка следующий: function [sys,x0,str,ts] = имя(t,x,u,flag);
% где "имя" - имя функции присвоенное блоку S-Function в модели симулинка. Оно
% обязательно должно соответсвовать имени М-файла, в котором эта функция содержиться.
% Остальные параметры являются обязательными и изменению не подлежат. Можно лишь добавить
% параметр после слова flag, но необходимо его "прописать" и в S-Function в модели симулинка
% Далее следуют флаги. Эта часть функции также обязательна. Управление  функцией
% осуществляется через флаги
% Когда флаг:
%		0 - инициализация функции - при первом ее запуске
%		2 - обновление внутренних переменных и состояний функции, т.е организация
%			 обновления по вектору входов вектора внутренних состояний.
%			 Выходная координата - матрица sys - соответсвует вектору x
%		3 - формирование выходных сигналов функции - через вектор sys
% В данном случае по каждому из флагов вызывается специальная функция.
% Причем, по выходному значению этой функции организуется управление
% остановом моделирования.
% Входы и выходы передаются через вектора!!!
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
% Возвращает размер, начальные условаия и так квантования вызова  для функции
function [sys,x0,str,ts]=mdlInitializeSizes(tq,decf)
%"Global" word need in that part of program (in this function)
% where global variable is used

% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%Обязательно!!!
sizes = simsizes; % получить те параметры, которые необходимо установить для
                  % адекватной работы S-функции

sizes.NumContStates  = 0; % Число непрерывных внутренних состояний
sizes.NumDiscStates  = 0; % Число дискретных внутренних состояний
sizes.NumOutputs     = 2; % Число выходов
sizes.NumInputs      = 2; % Число входов
sizes.DirFeedthrough = 1; % ???
sizes.NumSampleTimes = 1; % по меньшей мере один шаг квантования необходим
% at least one sample time is needed

sys = simsizes(sizes);
% initialize the initial conditions
% Задание начальных условий - для внутреннего вектора состояний
x0  = [];
% str is always an empty matrix
% str - всегда пустая матрица
str = [];
% initialize the array of sample times
% инициализация массива тактов квантования
% первый элемент - такт квантования
ts  = [tq*decf 0];

function sys=mdlOutputs(t,x,u,decf,uy);
sys(1) = u(1);
sys(2) = u(2)./uy;
