#include <stdio.h>
#include <math.h>

#include <portaudio.h>

#define SAMPLE_RATE 44100
#define AMPLITUDE 0.5

typedef struct {
    double frequency;
    double phase;
    double phaseIncrement;
    bool freqswitch;
} SineWaveData;

static int paCallback(const void *inputBuffer, void *outputBuffer, unsigned long framesPerBuffer, const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags, void *userData) {
    float *out = (float*)outputBuffer;
    SineWaveData *data = (SineWaveData*)userData;

    // if(data->freqswitch) {
    //     if(data->frequency < 880.0) {
    //         data->frequency += 1;
    //     } else if(data->frequency >= 880.0) {
    //         data->freqswitch = false;
    //     }
    // } else {
    //     if(data->frequency > 220.0) {
    //         data->frequency -= 1;
    //     } else if(data->frequency <= 220.0) {
    //         data->freqswitch = true;
    //     }
    // }
    
    for (unsigned long i = 0; i < framesPerBuffer; i++) {
        *out++ = (float)(AMPLITUDE * sin(data->phase));
        data->phase += (2.0 * M_PI * data->frequency) / SAMPLE_RATE;
        if (data->phase >= 2.0 * M_PI) {
            data->phase -= 2.0 * M_PI;
        }
    }
    return paContinue;
}

int main(void) {
    PaStream *stream;
    PaError err;
    SineWaveData data;
    data.frequency = 440.0;
    data.phase = 0.0;
    data.phaseIncrement = (2.0 * M_PI * data.frequency) / SAMPLE_RATE;
    data.freqswitch = true;

    err = Pa_Initialize();
    if (err != paNoError) goto error;

    err = Pa_OpenDefaultStream(&stream, 0, 1, paFloat32, SAMPLE_RATE, 256, paCallback, &data);
    if (err != paNoError) goto error;

    err = Pa_StartStream(stream);
    if (err != paNoError) goto error;

    getchar();

    while(true) {
        char key = getchar();
        if(key == 65) {
            data.frequency = 440.0; // A
            printf("A");
        } else if(key == 83) {
            data.frequency = 880.0; // S
            printf("S");
        } else if(key == 68) {
            data.frequency = 220.0; // D
            printf("D");
        }
    }

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
