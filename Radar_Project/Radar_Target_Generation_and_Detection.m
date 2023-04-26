clear all
clc ;

%% Radar Specifications 
%%%%%%%%%%%%%%%%%%%%%%%%%%%
Frequency= 77e9 %GHz
Max_Range = 200 %m
Range_Resolution = 1 %m
Max_Velocity = 100 %m/s
%%%%%%%%%%%%%%%%%%%%%%%%%%%

Speed_Of_Light = 3*10^8 %m/s
%% User Defined Range and Velocity of target
% *%TODO* :
% define the target's initial position and velocity. Note : Velocity
% remains contant
 
P_Int_1 = 20; %m
V_Int = -60; %m/s 

%% FMCW Waveform Generation

% *%TODO* :
%Design the FMCW waveform by giving the specs of each of its parameters.
% Calculate the Bandwidth (B), Chirp Time (Tchirp) and Slope (slope) of the FMCW
% chirp using the requirements above.

%Calculating the Bsweep 
B_sweep = Speed_Of_Light/2*Range_Resolution;  

%B_sweep will be =150 mega_Hz or 0.15 giga_Hz, therefore. the final
%frequency will be 77+0.15 = 77.15 giga_Hz the range of frequencies
%77:77.15 giga_Hz

%Calculating the Tchirp 
Tchirp = 5.5*2*Max_Range/Speed_Of_Light;

%Calculating the Slop
Slope = B_sweep/Tchirp; 

%Operating carrier frequency of Radar 
fc= 77e9;             %carrier freq
                                                          
%The number of chirps in one sequence. Its ideal to have 2^ value for the ease of running the FFT
%for Doppler Estimation. 
Nd=128;                   % #of doppler cells OR #of sent periods % number of chirps

%The number of samples on each chirp. 
Nr=1024;                  %for length of time OR # of range cells

% Timestamp for running the displacement scenario for every sample on each
% chirp
t=linspace(0,Nd*Tchirp,Nr*Nd); %total time for samples

%Creating the vectors for Tx, Rx and Mix based on the total samples input.
Tx=zeros(1,length(t)); %transmitted signal
Rx=zeros(1,length(t)); %received signal
Mix = zeros(1,length(t)); %beat signal

%Similar vectors for range_covered and time delay.
r_t=zeros(1,length(t));
td=zeros(1,length(t));

%% Signal generation and Moving Target simulation
% Running the radar scenario over the time. 

for i=1:length(t)          
    % *%TODO* :
    %For each time stamp update the Range of the Target for constant velocity. 
    P_Int = P_Int_1 + V_Int*t(i);
    % *%TODO* :
    %For each time sample we need update the transmitted and
    %received signal. 
    Tx(i)   = cos(2*pi*(fc*t(i) + Slope*t(i)^2/2));
    Rx (i)  = cos(2*pi*(fc*(t(i)-((2*P_Int)/Speed_Of_Light))+ Slope*(t(i)-((2*P_Int)/Speed_Of_Light))^2/2));
    
    % *%TODO* :
    %Now by mixing the Transmit and Receive generate the beat signal
    %This is done by element wise matrix multiplication of Transmit and
    %Receiver Signal
    Mix(i) = Tx(i)*Rx(i);
    
end
%% RANGE MEASUREMENT
 % *%TODO* :
%reshape the vector into Nr*Nd array. Nr and Nd here would also define the size of
%Range and Doppler FFT respectively.
Signal_r = reshape(Mix,[Nr, Nd]);

% *%TODO* :
%run the FFT on the beat signal along the range bins dimension (Nr) and
%normalize.
Signal_fft = fft(Signal_r);

% *%TODO* :
% Take the absolute value of FFT output
Signal_Abs = abs(Signal_fft/Nr);

 % *%TODO* :
% Output of FFT is double sided signal, but we are interested in only one side of the spectrum.
% Hence we throw out half of the samples.
 Signal_Final = Signal_Abs(1:Nr/2+1);

%plotting the range
figure ('Name','Range from First FFT')
subplot(3,1,1)
plot(Signal_Final);
axis ([0 200 0 0.5]);
xlabel('Distance in M');
ylabel('Power of signal in dB');
subplot(3,1,2)
plot(1000*(0:Nr/2)/Nr,Signal_Final);
xlabel('Beat Frequency with sampling Frequency 1000 Hz');
ylabel('Power of signal in dB');
axis ([0 200 0 0.5]);


%% RANGE DOPPLER RESPONSE
% The 2D FFT implementation is already provided here. This will run a 2DFFT
% on the mixed signal (beat signal) output and generate a range doppler
% map.You will implement CFAR on the generated RDM


% Range Doppler Map Generation.

% The output of the 2D FFT is an image that has reponse in the range and
% doppler FFT bins. So, it is important to convert the axis from bin sizes
% to range and doppler based on their Max values.

Mix=reshape(Mix,[Nr,Nd]);

% 2D FFT using the FFT size for both dimensions.
sig_fft2 = fft2(Mix,Nr,Nd);
si = sig_fft2;
% Taking just one side of signal from Range dimension.
sig_fft2 = sig_fft2(1:Nr/2,1:Nd);
sig_fft2 = fftshift (sig_fft2);
RDM = abs(sig_fft2);

%Plot the Velocity and the Range of the target in imagesc  
V = linspace(-Nd/2,Nd/2-1,Nd)*2*Max_Velocity/Nd;
r = linspace(-Nr/2,Nr/2-1,Nr)*2*Max_Range/Nr;
subplot(3,1,3);
imagesc(V,r,RDM);
xlabel('Velcoity in M/s');
ylabel('Range in M');
%colormap jet;
colorbar;
%clim([0,max(RDM(:))]);
RDM = 10*log10(RDM) ;

%use the surf function to plot the output of 2DFFT and to show axis in both
%dimensions
doppler_axis = linspace(-100,100,Nd);
range_axis = linspace(-200,200,Nr/2)*((Nr/2)/400);
figure,surf(doppler_axis,range_axis,RDM);
xlabel('Velcoity in M/s');
ylabel('Range in M');
%% CFAR implementation

%Slide Window through the complete Range Doppler Map

% *%TODO* :
%Select the number of Training Cells in both the dimensions.

Tr = 12;
Td = 12;

% *%TODO* :
%Select the number of Guard Cells in both dimensions around the Cell under 
%test (CUT) for accurate estimation
Gr = 4;
Gd = 4;

% *%TODO* :
% offset the threshold by SNR value in dB
Off_S = 5;

%% Size of Training, Grid, and  Guard Cells 
Training_Size = ((2*Tr)+(2*Gr)+1)*((2*Td)+(2*Gd)+1) - ((2*Gr)+1)*((2*Gd)+1);
Grid_Size = ((2*Tr)+(2*Gr)+1)*((2*Td)+(2*Gd)+1);
Guard_Size = (2*Gr+1)*(2*Gd+1);

% *%TODO* :
%Create a vector to store noise_level for each iteration on training cells
noise_level = zeros(1,1);

% *%TODO* :
%design a loop such that it slides the CUT across range doppler map by
%giving margins at the edges for Training and Guard Cells.
%For every iteration sum the signal level within all the training
%cells. To sum convert the value from logarithmic to linear using db2pow
%function. Average the summed values for all of the training
%cells used. After averaging convert it back to logarithimic using pow2db.
%Further add the offset to it to determine the threshold. Next, compare the
%signal under CUT with this threshold. If the CUT level > threshold assign
%it a value of 1, else equate it to 0.

zer = zeros(size(RDM));

% Use RDM[x,y] as the matrix from the output of 2D FFT for implementing
% CFAR
for i = 1+Tr+Gr:(size(RDM,1) - (Gr+Tr))
    for j = 1+Td+Gd:(size(RDM,2) - (Gd+Td))
        sum_all = sum(db2pow(RDM(i-(Tr+Gr):i+Tr+Gr, j-(Td+Gd):j+Td+Gd)),'all');
        T_sum  = sum(db2pow(RDM(i-Gr:i+Gr, j-Gd:j+Gd)),'all');
        noise_level = sum_all - T_sum;
        threshold = db2pow(pow2db((noise_level / Training_Size)*Off_S));
        if (db2pow(RDM(i,j)) <= threshold)
            zer(i,j) = 0;
        else
            zer(i,j) = 1;
        end
    end
end

% *%TODO* :
%display the CFAR output using the Surf function like we did for Range
%Doppler Response output.
figure,surf(doppler_axis,range_axis,zer);
xlabel('Velcoity in M/s');
ylabel('Range in M');
colorbar;


 
 