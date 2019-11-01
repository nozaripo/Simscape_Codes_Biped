function [tau, ym, omega, y] = Trajectory(varargin)
St = load('Baseline.mat');

% tt1 = 1
% tt=113;
rep = 2;


% tt1 = 31;     % 31:31+55
tt1 = varargin{2};
tt2 = tt1+113;

tt3 = tt2+1;  % 31+56:tt
tt4 = tt3+55;


rep = 1;


% 
if varargin{1} == 1
    y = repmat(St.Baseline.L_Hip_Angle(tt1:tt2,1),rep,1);
elseif varargin{1}==2
    y = repmat(St.Baseline.L_Knee_Angle(tt1:tt2,1),rep,1);
elseif varargin{1}==3
    y = repmat(St.Baseline.L_Ankle_Angle(tt1:tt2,1),rep,1);
elseif varargin{1}==4
    y = repmat(St.Baseline.R_Hip_Angle(tt1:tt2,1),rep,1);
elseif varargin{1}==5
    y = repmat(St.Baseline.R_Knee_Angle(tt1:tt2,1),rep,1);    
elseif varargin{1}==6
    y = repmat(St.Baseline.R_Ankle_Angle(tt1:tt2,1),rep,1);  
end


% if varargin{1} == 1
%     y = repmat(St.Baseline.L_Hip_Angle(tt1:tt2,1),rep,1);
% elseif varargin{1}==2
%     y = repmat(St.Baseline.L_Knee_Angle(tt1:tt2,1),rep,1);
% elseif varargin{1}==3
%     y = repmat(St.Baseline.L_Ankle_Angle(tt1:tt2,1),rep,1);
% elseif varargin{1}==4
%     y = repmat(St.Baseline.L_Hip_Angle(tt3:tt4,1),rep,1);
% elseif varargin{1}==5
%     y = repmat(St.Baseline.L_Knee_Angle(tt3:tt4,1),rep,1);    
% elseif varargin{1}==6
%     y = repmat(St.Baseline.L_Ankle_Angle(tt3:tt4,1),rep,1);  
% end




% if varargin{1} == 1
%     y = St.Baseline.L_Hip_Angle(1:tt,1);
% elseif varargin{1}==2
%     y = St.Baseline.L_Knee_Angle(1:tt,1);
% elseif varargin{1}==3
%     y = St.Baseline.L_Ankle_Angle(1:tt,1);
% elseif varargin{1}==4
%     y = St.Baseline.R_Hip_Angle(1:tt,1);
% elseif varargin{1}==5
%     y = St.Baseline.R_Knee_Angle(1:tt,1);    
% elseif varargin{1}==6
%     y = St.Baseline.R_Ankle_Angle(1:tt,1);  
% end




% if varargin{1} == 1
%     y = repmat(St.Baseline.L_Hip_Angle(60:60+tt-1,1),2,1);
% elseif varargin{1}==2
%     y = repmat(St.Baseline.L_Knee_Angle(60:60+tt-1,1),2,1);
% elseif varargin{1}==3
%     y = repmat(St.Baseline.L_Ankle_Angle(60:60+tt-1,1),2,1);
% elseif varargin{1}==4
%     y = repmat(St.Baseline.R_Hip_Angle(60:60+tt-1,1),2,1);
% elseif varargin{1}==5
%     y = repmat(St.Baseline.R_Knee_Angle(60:60+tt-1,1),2,1);    
% elseif varargin{1}==6
%     y = repmat(St.Baseline.R_Ankle_Angle(60:60+tt-1,1),2,1);  
% end

dt=.01;

T=y;
   
   Td               = diff(T)/dt;
Td               = [Td; Td(end)];

Tdd              = diff(Td)/dt;
Tdd              = [Tdd; Tdd(end)];

   
%%%%%% low pass filter cutoff 10Hz
d = fdesign.lowpass('N,Fc',20,.5,100);
   Hd = design(d);
   
   yy = filter(Hd,y);
   
    yyd              = diff(yy)/dt;
yyd               = [yyd; yyd(end)];

yydd              = diff(yyd)/dt;
yydd              = [yydd; yydd(end)];

   
%    figure (3)
%    subplot(311)
%    plot([y, yy])
%    
%    subplot(312)
%    plot([Td, yyd])
%    
%    subplot(313)
%    plot([Tdd, yydd])
   
   

Fs = 100;            % Sampling frequency
T = 1/Fs;             % Sampling period
% % % L = length(y);             % Length of signal
% % % t = (1:L-1)*T;        % Time vector
% % % 
% % % FF = fft(y);
% % % 
% % % P2 = abs(FF/L);
% % % P1 = P2(1:L/2+1);
% % % P1(2:end-1) = 2*P1(2:end-1);

% % % f = Fs*(0:(L/2))/L;
% figure (1)
% plot(f,P1) 
% title('Single-Sided Amplitude Spectrum of X(t)')
% xlabel('f (Hz)')
% ylabel('|P1(f)|')

ac=xcorr(y,y);
[~,locs]=findpeaks(ac);
omega=mean(diff(locs)*T);
tau = (2*pi)/omega;

ym=mean(y)/2;
ym = y(1,1);

end