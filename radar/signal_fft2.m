


P = peaks(20);
X = repmat(P,[5 10]);
imagesc(X);

M = 100;
N = 200;

signal  = reshape(X, [M, N]);

signal_fft = fft2(signal, M, N);

signal_fft = fftshift(signal_fft);

signal_fft = abs(signal_fft);

imagesc(signal_fft);