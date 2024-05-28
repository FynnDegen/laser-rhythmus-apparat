#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <regex>
#include <stdio.h>
#include <math.h>

#include <portaudio.h>

#define SAMPLE_RATE 44100

typedef struct {
    double frequency;
    double amplitude;
    double phase;
    double phaseIncrement;
} SineWaveData;

static int paCallback(const void *inputBuffer, void *outputBuffer, unsigned long framesPerBuffer, const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags, void *userData) {
    float *out = (float*)outputBuffer;
    SineWaveData *data = (SineWaveData*)userData;
    
    for (unsigned long i = 0; i < framesPerBuffer; i++) {
        *out++ = (float)(data->amplitude * sin(data->phase));
        data->phase += (2.0 * M_PI * data->frequency) / SAMPLE_RATE;
        if (data->phase >= 2.0 * M_PI) {
            data->phase -= 2.0 * M_PI;
        }
    }
    return paContinue;
}

int main() {
    // serial port
    int serial_port = open("/dev/ttyACM0", O_RDONLY);

    //PortAudio
    PaStream *stream;
    PaError err;
    SineWaveData data;
    data.frequency = 440.0;
    data.amplitude = 0.5;
    data.phase = 0.0;
    data.phaseIncrement = (2.0 * M_PI * data.frequency) / SAMPLE_RATE;

    err = Pa_Initialize();
    if (err != paNoError) goto error;

    err = Pa_OpenDefaultStream(&stream, 0, 1, paFloat32, SAMPLE_RATE, 256, paCallback, &data);
    if (err != paNoError) goto error;

    err = Pa_StartStream(stream);
    if (err != paNoError) goto error;

    if (serial_port < 0) {
        std::cerr << "Error opening serial port\n";
        return 1;
    }

    // Configure serial port
    struct termios tty;
    if(tcgetattr(serial_port, &tty) != 0) {
        std::cerr << "Error getting termios attributes\n";
        close(serial_port);
        return 1;
    }

    tty.c_cflag &= ~PARENB; // No parity bit
    tty.c_cflag &= ~CSTOPB; // One stop bit
    tty.c_cflag &= ~CSIZE;  // Clear size bits
    tty.c_cflag |= CS8;     // 8 data bits
    tty.c_cflag &= ~CRTSCTS; // No flow control
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    tty.c_cc[VTIME] = 1;    // Wait for up to 1s, returning as soon as any data is received
    tty.c_cc[VMIN] = 0;

    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting termios attributes\n";
        close(serial_port);
        return 1;
    }

    // Read and display data
    char read_buf[256];
    memset(&read_buf, '\0', sizeof(read_buf));

    while (true) {
        int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

        if (num_bytes < 0) {
            std::cerr << "Error reading from serial port\n";
            break;
        }

        // std::string pattern(":1:[1-9][0-9]*:2:[1-9][0-9]*");
        // std::regex rx(pattern); 
        // std::string measurement = atoi(read_buf);
        int measurement = atoi(read_buf);
        if(measurement <= 880 && measurement >= 220) {
            data.frequency = measurement;
        } else if((measurement > 880 || measurement < 220) && measurement != 0) {
            data.frequency = 0;
        }

        std::cout << measurement << std::endl;
        memset(&read_buf, '\0', sizeof(read_buf));
    }

    close(serial_port);

    err = Pa_StopStream(stream);
    if (err != paNoError) goto error;

    err = Pa_CloseStream(stream);
    if (err != paNoError) goto error;

    Pa_Terminate();
    return 0;

error:
    Pa_Terminate();
    fprintf(stderr, "An error occurred while using the PortAudio stream\n");
    fprintf(stderr, "Error number: %d\n", err);
    fprintf(stderr, "Error message: %s\n", Pa_GetErrorText(err));
    return 1;
}
