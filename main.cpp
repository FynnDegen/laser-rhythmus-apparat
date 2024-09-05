#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <signal.h>
#include <random>

#include <portaudio.h>

#define MODE true

#define SAMPLE_RATE 44100
#define NUM_LASER 5

struct SineWaveData {
    double frequency = 0.0;
    double amplitude = 0.0;
    double phase = 0.0;
    bool isPlayed = false;
    short osc = 0;

    void increment() {
        phase += (2.0 * M_PI * frequency) / SAMPLE_RATE;
        if (phase >= 2.0 * M_PI) {
            phase -= 2.0 * M_PI;
        }
    }
};

int serialPort;

PaStream *stream;
SineWaveData data[NUM_LASER];

double oscillator(int oscNum, double phase) {
    switch(oscNum) {
        case 0:
            return sin(phase); // sine wave
        case 1:
            return (phase < 0.5) ? 0 : 1; // square wave
        case 2:
            return (2/M_PI)*asin(sin(phase));
        case 3:
            return 2*((phase/2*M_PI) - floor(0.5 + (phase/2*M_PI)));
        default:
            return 0.0;
    }
}

static int paCallback(const void *inputBuffer, void *outputBuffer, unsigned long framesPerBuffer, const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags, void *userData) {
    float *out = (float*)outputBuffer;
    SineWaveData *data = (SineWaveData*)userData;
    
    for (unsigned long i = 0; i < framesPerBuffer; i++) {

        float audioOutput = 0;
        for(size_t i = 0; i < NUM_LASER; i++) {
            audioOutput += data[i].amplitude * oscillator(data[i].osc, data[i].phase);
        }
        audioOutput /= 3.75;
        *out++ = audioOutput;

        for(size_t i = 0; i < NUM_LASER; i++) {
            data[i].increment();
        }
    }
    return paContinue;
}

void checkErr(PaError err) {
    Pa_Terminate();
    std::cerr << "Fehler bei der Nutzung von PortAudio: " << Pa_GetErrorText(err) << std::endl;
    exit(-1);
}

int openSerialPort(const char* port) {
    int serialPort = open(port, O_RDONLY);

    if(serialPort < 0) {
        std::cerr << "Fehler beim Verbinden zum Arduino" << std::endl;
        exit(-1);
    }

    return serialPort;
}

void defaultMode() {

    data[0].frequency = 987.767; // h2
    data[1].frequency = 783.991; // g2
    data[2].frequency = 659.255; // e2
    data[3].frequency = 554.365; // cis2/des2
    data[4].frequency = 440.0;   // a1

    char read_buf[256];

    while(true) {
        memset(&read_buf, '\0', sizeof(read_buf));
        int num_bytes = read(serialPort, &read_buf, sizeof(read_buf));

        if(num_bytes < 0) {
            std::cerr << "Fehler beim Lesen vom Arduino" << std::endl;
            break;
        }

        std::string measurement = read_buf;
        if(measurement.find(":") != std::string::npos) {
            int laserIndex = measurement.at(1) - '0';
            int laser = std::stoi(measurement.substr(3));

            std::cout << laserIndex << " " << laser << std::endl;

            if(laser <= 720 && laser >= 60) {
                if(data[laserIndex].isPlayed) {
                    data[laserIndex].amplitude = std::max(0.5+((double)(rand() % 10)) / 100.0, data[laserIndex].amplitude-0.125);
                } else {
                    data[laserIndex].amplitude = 0.75;
                    data[laserIndex].isPlayed = true;
                }
            } else if((laser > 720 || laser < 60) && laser != 0) {
                data[laserIndex].amplitude = std::max(0.0, data[laserIndex].amplitude-0.075);
                data[laserIndex].isPlayed = false;
            }
        }
    }
}

void experimentalMode() {

    char read_buf[256];

    while(true) {
        memset(&read_buf, '\0', sizeof(read_buf));
        int num_bytes = read(serialPort, &read_buf, sizeof(read_buf));

        if(num_bytes < 0) {
            std::cerr << "Fehler beim Lesen vom Arduino" << std::endl;
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
    if(PaError err = Pa_StopStream(stream) != paNoError) {
        checkErr(err);
    }
    if(PaError err = Pa_CloseStream(stream) != paNoError) {
        checkErr(err);
    }
    if(PaError err = Pa_Terminate() != paNoError) {
        checkErr(err);
    }
    close(serialPort);
    std::cout << "\nLRA beendet" << std::endl;
    exit(0);
}

int main() {
    // Signal handler
    signal(SIGINT, signalHandler);

    // Serial port
    serialPort = openSerialPort("/dev/ttyACM0");

    //PortAudio
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