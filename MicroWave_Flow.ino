#include "arduinoFFT.h"
#include "RPi_Pico_TimerInterrupt.h"

const int adc2_pin = 28;                // GPIO28 for ADC2 (I component)
const int adc3_pin = 29;                // GPIO29 for ADC3 (Q component)
const int led_pin = 25;                 // GPIO25 for LED
const uint16_t samples = 512;           // Must be a power of 2
const float samplingFrequency = 10000;  // Desired sampling frequency in Hz
unsigned long microseconds;
unsigned int sampling_period_us;

float vReal[samples];                    // In-phase (I) component
float vImag[samples];                    // Quadrature (Q) component

ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, samples, samplingFrequency, true);
bool ledState = LOW;

const float MIN_FLOW_RATE = 0;          // Minimum expected flow rate
const float MAX_FLOW_RATE = 80;         // Maximum expected flow rate
const int OUTPUT_MIN = 1;                // Minimum output value
const int OUTPUT_MAX = 4000;             // Maximum output value
const int calibrationFactor = 1;
float maxPSD = 0;                        // Maximum observed PSD value
float backgroundReal[samples];            // Background FFT (I component)
float backgroundImag[samples];            // Background FFT (Q component)
bool isBackgroundCaptured = false;

// Index for the current sample
volatile int currentSampleIndex = 0;

// Timer interrupt handler
bool TimerHandler0(struct repeating_timer *t) {
    (void)t;
    static bool toggle0 = false;
    digitalWrite(led_pin, toggle0);
    toggle0 = !toggle0;

    // Sample data from ADC
    if (currentSampleIndex < samples) {
        vReal[currentSampleIndex] = analogRead(adc2_pin) << 4;  // Read from ADC2 (I component)
        vImag[currentSampleIndex] = analogRead(adc3_pin) << 4;  // Read from ADC3 (Q component)
        currentSampleIndex++;
    } else {
        // Reset the index when done sampling
        currentSampleIndex = 0;
    }

    return true;
}

// Adjust timer interval based on desired sampling frequency
#define TIMER0_INTERVAL_MS 0.1  // 10 kHz sampling frequency

// Init RPI_PICO_Timer
RPI_PICO_Timer ITimer0(0);

void setup() {
    sampling_period_us = round(1000000 * (1.0 / samplingFrequency));  // Calculate sampling period
    Serial.begin(115200);
    Serial.println("Ready");
    pinMode(led_pin, OUTPUT);

    // Initialize the FFT arrays
    memset(vReal, 0, sizeof(vReal));
    memset(vImag, 0, sizeof(vImag));
    memset(backgroundReal, 0, sizeof(backgroundReal));
    memset(backgroundImag, 0, sizeof(backgroundImag));

    // Capture the initial background FFT
    captureBackgroundFFT();

    // Attach timer interrupt
    if (ITimer0.attachInterruptInterval(TIMER0_INTERVAL_MS * 1000, TimerHandler0)) {
        Serial.print(F("Starting ITimer0 OK, millis() = ")); Serial.println(millis());
    } else {
        Serial.println(F("Can't set ITimer0. Select another Timer, freq. or timer"));
    }
}

void loop() {
    // Check if we have finished sampling all data
    if (currentSampleIndex >= samples) {
        // Perform FFT
        FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);  // Apply windowing to the signal
        FFT.compute(FFTDirection::Forward);                        // Compute FFT
        FFT.complexToMagnitude();                                  // Convert to magnitude

        // Adaptive Noise Cancellation using background FFT
        adaptiveNoiseCancellation(vReal, vImag, backgroundReal, backgroundImag, samples);

        // Calculate dominant frequency
        float dominantFrequency = FFT.majorPeak();

        // Calculate Power Spectral Density
        float psd = calculatePowerSpectralDensity(vReal, vImag, samples);

        // Update maximum PSD value if the current one is greater
        if (psd > maxPSD) {
            maxPSD = psd;
        }

        // Normalize PSD to range [0, 1]
        float normalizedPSD = psd / maxPSD;

        // Scale the flow rate to a range of 1 to 4000
        int ADC = mapADC(dominantFrequency, normalizedPSD);

        // Output results
        Serial.print("Dominant Frequency: ");
        Serial.print(dominantFrequency, 6);
        Serial.print(" Hz, Scaled ADC: ");
        Serial.println(ADC);  // Output the scaled flow rate

        // Reset currentSampleIndex for the next batch of samples
        currentSampleIndex = 0;
    }
}

// Function to capture the background FFT
void captureBackgroundFFT() {
    // Collect initial samples to define background
    for (int i = 0; i < samples; i++) {
        backgroundReal[i] = analogRead(adc2_pin) << 4;
        backgroundImag[i] = analogRead(adc3_pin) << 4;

        // Wait for the next sample
        delayMicroseconds(sampling_period_us);
    }

    // Perform FFT on the background samples
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();
}

// Adaptive noise cancellation function
void adaptiveNoiseCancellation(float *vReal, float *vImag, float *backgroundReal, float *backgroundImag, uint16_t samples) {
    float noise_threshold = 0.1;  // Threshold for noise cancellation

    // Subtract background FFT from current FFT
    for (int i = 0; i < samples; i++) {
        vReal[i] -= backgroundReal[i];
        vImag[i] -= backgroundImag[i];
    }

    // Zero out values below the noise threshold
    for (int i = 0; i < samples; i++) {
        float magnitude = sqrt(vReal[i] * vReal[i] + vImag[i] * vImag[i]);
        if (magnitude < noise_threshold) {
            vReal[i] = 0;
            vImag[i] = 0;
        }
    }
}

// Function to calculate Power Spectral Density
float calculatePowerSpectralDensity(float *vReal, float *vImag, uint16_t samples) {
    float psd = 0.0;
    for (int i = 0; i < samples; i++) {
        float magnitude = sqrt(vReal[i] * vReal[i] + vImag[i] * vImag[i]);
        psd += magnitude * magnitude;  // Accumulate power
    }
    return psd / samples;  // Return average PSD
}

// Function to scale flow rate
int mapADC(float dominantFrequency, float normalizedPSD) {
    // Convert the dominant frequency to a flow rate using calibration factor
    float ADC = calibrationFactor * dominantFrequency;

    // Optionally include a contribution from the normalized PSD to the flow rate
    ADC += normalizedPSD * (OUTPUT_MAX - OUTPUT_MIN) * 0.01;  // Adjust this factor as necessary

    // Clamp ADC to the defined range
    if (ADC < MIN_FLOW_RATE) {
        ADC = MIN_FLOW_RATE;
    } else if (ADC > MAX_FLOW_RATE) {
        ADC = MAX_FLOW_RATE;
    }

    // Scale the flow rate to the range 1-4000
    int scaledADC = (ADC - MIN_FLOW_RATE) * (OUTPUT_MAX - OUTPUT_MIN) / (MAX_FLOW_RATE - MIN_FLOW_RATE) + OUTPUT_MIN;
    return scaledADC;
}
