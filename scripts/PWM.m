
hold on
plot(measure.time,measure.pwm);
plot(measure.time,measure.pwm_jtc,'r');
%plot(measure.time,measure.pwm./measure.pwm_jtc,'g');
xlabel('time','Interpreter','tex');
ylabel('PWM','Interpreter','tex');
grid;
hold off
Tc = 0.01;
Fs = 1/Tc;                    % Sampling frequency
L = size(measure.time,1);       % Length of signal

NFFT = 2^nextpow2(L); % Next power of 2 from length of y
Y = fft(measure.pwm,NFFT)/L;
Y2 = fft(measure.pwm_jtc,NFFT)/L;
f = Fs/2*linspace(0,1,NFFT/2+1);

figure;
% Plot single-sided amplitude spectrum.
semilogx(f,2*abs(Y(1:NFFT/2+1)));
hold on;
semilogx(f,2*abs(Y2(1:NFFT/2+1)),'r');
%semilogx(f,2*abs(Y3(1:NFFT/2+1)),'g');
grid;
hold off;
title('Single-Sided Amplitude Spectrum of y(t)')
xlabel('Frequency (Hz)')
ylabel('|Y(f)|')