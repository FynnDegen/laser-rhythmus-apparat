#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#include <portaudio.h>

static void checkErr(PaError err) {
    if(err != paNoError) {
        std::cout << "PortAudio error: " << Pa_GetErrorText(err) << std::endl;
        exit(EXIT_FAILURE);
    }
}

int main() {
    std::cout << "Start" << std::endl;

    PaError err = Pa_Initialize();
    checkErr(err);

    int numDevices = Pa_GetDeviceCount();
    std::cout << numDevices << std::endl;

    const PaDeviceInfo* deviceInfo;
    for(int i = 0; i < numDevices; i++) {
        deviceInfo = Pa_GetDeviceInfo(i);
        std::cout << "\nDevice(" << i << ", " << deviceInfo->name << "):\n\tmaxInputChannels: " << deviceInfo->maxInputChannels << "\n\tmaxOuputChannels: " << deviceInfo->maxOutputChannels << "\n\tdefaultSampleRate: " << deviceInfo->defaultSampleRate << std::endl;
    }

    err = Pa_Terminate();
    checkErr(err);

    return EXIT_SUCCESS;
}