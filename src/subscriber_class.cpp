#include <portaudio.h>
#include <stdio.h>
#include <stdlib.h>

#include <functional>
#include <iostream>
#include <limits>
#include <math.h>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

#define SAMPLE_RATE   (44100)
#define FRAMES_PER_BUFFER  (64)

#ifndef M_PI
#define M_PI  (3.14159265)
#endif

#define TABLE_SIZE   (200)
typedef struct {
    float sine[TABLE_SIZE];
    int left_phase;
    int right_phase;
    char message[20];
    float angle;
}
paTestData;

using std::placeholders::_1;

class MySubscriber : public rclcpp::Node {

   private:
    PaStreamParameters outputParameters;
    PaStream *stream;
    PaError err;
    paTestData data;

   public:
    MySubscriber() : Node("subscriber") {
        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", default_qos, std::bind(&MySubscriber::topic_callback, this, _1));

        for(int i = 0; i<TABLE_SIZE; i++ ) {
            data.sine[i] = (float) sin( ((double)i/(double)TABLE_SIZE) * M_PI * 2. );
        }
        data.left_phase = data.right_phase = 0;

        PaError err = Pa_Initialize();
        checkErr(err);

        outputParameters.device = Pa_GetDefaultOutputDevice(); /* default output device */
        outputParameters.channelCount = 2;       /* stereo output */
        outputParameters.sampleFormat = paFloat32; /* 32 bit floating point output */
        outputParameters.suggestedLatency = Pa_GetDeviceInfo( outputParameters.device )->defaultLowOutputLatency;
        outputParameters.hostApiSpecificStreamInfo = NULL;

        err = Pa_OpenStream(&stream, NULL, &outputParameters, SAMPLE_RATE, FRAMES_PER_BUFFER, paClipOff, patestCallback, &data );
        checkErr(err);
    }

   private:
    void topic_callback(const sensor_msgs::msg::LaserScan& msg) {
        float min = msg.ranges[0];
        for(size_t i = 1; i < msg.ranges.size(); i++) {
            if(msg.ranges[i] < min && msg.ranges[i] > msg.range_min ) {
                min = msg.ranges[i];
                data.angle = std::abs(msg.angle_min + msg.angle_increment*i*2);
            }
        }
        if(min < 3.0f) {
            Pa_StartStream(stream);
        } else {
            Pa_StopStream(stream);
        }
        std::cout << min << " : " << data.angle << std::endl;
    }

    static int patestCallback(const void *inputBuffer, void *outputBuffer, unsigned long framesPerBuffer, const PaStreamCallbackTimeInfo *timeInfo, PaStreamCallbackFlags statusFlags, void *userData) {
        paTestData *data = (paTestData*)userData;
        float *out = (float*)outputBuffer;
        (void) timeInfo; /* Prevent unused variable warnings. */
        (void) statusFlags;
        (void) inputBuffer;

        for(unsigned long i = 0; i < framesPerBuffer; i++ ) {
            *out++ = data->sine[data->left_phase];  /* left */
            *out++ = data->sine[data->right_phase];  /* right */
            data->left_phase += data->angle;
            if( data->left_phase >= TABLE_SIZE ) data->left_phase -= TABLE_SIZE;
            data->right_phase += data->angle;
            if( data->right_phase >= TABLE_SIZE ) data->right_phase -= TABLE_SIZE;
        }

        return paContinue;
    }
    
    static void StreamFinished( void* userData ) {
        paTestData *data = (paTestData *) userData;
        printf( "Stream Completed: %s\n", data->message );
    }

    static void checkErr(PaError err) {
        if(err != paNoError) {
            std::cout << "PortAudio error: " << Pa_GetErrorText(err) << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MySubscriber>());
    rclcpp::shutdown();

    return 0;
}