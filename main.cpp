#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <signal.h>

#include <portaudio.h>

#define MODE true

#define SAMPLE_RATE 44100
#define NUM_LASER 5

struct SineWaveData {
    double frequency;
    double amplitude;
    double phase;

    void increment() {
        phase += (2.0 * M_PI * frequency) / SAMPLE_RATE;
        if (phase >= 2.0 * M_PI) {
            phase -= 2.0 * M_PI;
        }
    }
};

int serial_port;

PaStream *stream;
SineWaveData data[NUM_LASER];

static int paCallback(const void *inputBuffer, void *outputBuffer, unsigned long framesPerBuffer, const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags, void *userData) {
    float *out = (float*)outputBuffer;
    SineWaveData *data = (SineWaveData*)userData;
    
    for (unsigned long i = 0; i < framesPerBuffer; i++) {

        float audioOutput = 0;
        for(size_t i = 0; i < NUM_LASER; i++) {
            audioOutput += data[i].amplitude * sin(data[i].phase);
        }
        audioOutput /= 3;
        *out++ = audioOutput;

        for(size_t i = 0; i < NUM_LASER; i++) {
            data[i].increment();
        }
    }
    return paContinue;
}

void checkErr(PaError err) {
    Pa_Terminate();
    fprintf(stderr, "An error occurred while using the PortAudio stream\n");
    fprintf(stderr, "Error number: %d\n", err);
    fprintf(stderr, "Error message: %s\n", Pa_GetErrorText(err));
    exit(-1);
}

void initalizeSineWaveData(SineWaveData data[]) {
    for(int i = 0; i < NUM_LASER; i++) {
        data[i].frequency = 0.0;
        data[i].amplitude = 0.5;
        data[i].phase = 0.0;
    }
}

int openSerialPort(const char* port) {
    int serialPort = open(port, O_RDONLY);

    if(serialPort < 0) {
        std::cerr << "Error opening serial port\n";
        return -1;
    }

    // Configure serial port
    struct termios tty;
    if(tcgetattr(serialPort, &tty) != 0) {
        std::cerr << "Error getting termios attributes\n";
        close(serialPort);
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

    if(tcsetattr(serialPort, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting termios attributes\n";
        close(serialPort);
        return 1;
    }

    return serialPort;
}

void defaultMode() {

    const float notes[5] = {
        987.767, 783.991, 659.255, 554.365, 440.0
    }; // h2 g2 e2 cis2/des2 a1

    char read_buf[256];

    while(true) {
        memset(&read_buf, '\0', sizeof(read_buf));
        int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

        if(num_bytes < 0) {
            std::cerr << "Error reading from serial port\n";
            break;
        }

        std::string measurement = read_buf;
        if(measurement.find(":") != std::string::npos) {
            int laserIndex = measurement.at(1) - '0';
            int laser = std::stoi(measurement.substr(3));

            std::cout << laserIndex << " " << laser << std::endl;

            if(laser <= 720 && laser >= 60) {
                data[laserIndex].frequency = notes[laserIndex];
            } else if((laser > 720 || laser < 60) && laser != 0) {
                data[laserIndex].frequency = 0;
            }
        }
    }
}

void experimentalMode() {

    char read_buf[256];

    while(true) {
        memset(&read_buf, '\0', sizeof(read_buf));
        int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

        if(num_bytes < 0) {
            std::cerr << "Error reading from serial port\n";
            break;
        }

        std::string measurement = read_buf;
        if(measurement.find(":") != std::string::npos) {
            int laserIndex = measurement.at(1) - '0';
            int laser = std::stoi(measurement.substr(3));

            std::cout << laserIndex << " " << laser << std::endl;

            if(laser <= 720 && laser >= 60) {
                data[laserIndex].frequency = laser + 160;
            } else if((laser > 720 || laser < 60) && laser != 0) {
                data[laserIndex].frequency = 0;
            }
        }
    }
}

void signalHandler(int signum) {
    close(serial_port);
    if(PaError err = Pa_StopStream(stream) != paNoError) {
        checkErr(err);
    }
    if(PaError err = Pa_CloseStream(stream) != paNoError) {
        checkErr(err);
    }
    Pa_Terminate();
    std::cout << "\nLRA beendet" << std::endl;
    exit(0);
}

int main() {
    // Signal handler
    signal(SIGINT, signalHandler);

    // Serial port
    serial_port = openSerialPort("/dev/ttyACM0");

    //PortAudio
    initalizeSineWaveData(data);

    if(PaError err = Pa_Initialize() != paNoError) {
        checkErr(err);
    }

    if( PaError err = Pa_OpenDefaultStream(&stream, 0, 1, paFloat32, SAMPLE_RATE, 256, paCallback, &data) != paNoError) {
        checkErr(err);
    }
    if(PaError err = Pa_StartStream(stream) != paNoError) {
        checkErr(err);
    }

    if(MODE) {
        defaultMode();
    } else {
        experimentalMode();
    }
}