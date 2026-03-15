#include "../core/FlightController.h"
#include "../core/HAL.h"
#include "../core/board/TeensyHAL.h"

#include <Arduino.h>

namespace {

class LegacyHalAdapter final : public HAL {
public:
    explicit LegacyHalAdapter(TeensyHAL& board) : board_(board) {}

    uint32_t micros() override { return static_cast<uint32_t>(board_.microsNow()); }
    uint32_t millis() override { return ::millis(); }
    void delay(uint32_t ms) override { ::delay(ms); }

    void serialBegin(uint32_t baud) override { Serial.begin(baud); }
    void serialPrint(const char* str) override { Serial.print(str); }
    void serialPrintln(const char* str) override { Serial.println(str); }
    void serialPrintFloat(float value, int decimals = 2) override { Serial.print(value, decimals); }

    bool initIMU() override {
        return board_.init();
    }

    bool readIMU(float* gyro_xyz, float* accel_xyz, float* mag_xyz) override {
        SensorData sensors;
        if (!board_.readSensors(sensors)) {
            return false;
        }
        gyro_xyz[0] = sensors.p;
        gyro_xyz[1] = sensors.q;
        gyro_xyz[2] = sensors.r;

        accel_xyz[0] = 0.0f;
        accel_xyz[1] = 0.0f;
        accel_xyz[2] = -9.80665f;

        mag_xyz[0] = 1.0f;
        mag_xyz[1] = 0.0f;
        mag_xyz[2] = 0.0f;
        return true;
    }

    bool isIMUHealthy() override {
        SensorData sensors;
        return board_.readSensors(sensors);
    }

    bool initRadio() override { return true; }

    bool readRadio(float* channels, int num_channels) override {
        PilotInput input;
        if (!board_.readPilotInput(input) || num_channels < 4) {
            return false;
        }

        channels[0] = input.throttle;
        channels[1] = input.roll;
        channels[2] = input.pitch;
        channels[3] = input.yaw;
        if (num_channels > 4) {
            channels[4] = input.armed ? 1.0f : -1.0f;
        }
        if (num_channels > 5) {
            channels[5] = 0.0f;
        }
        return true;
    }

    bool isRadioConnected() override { return true; }

    bool initServos() override { return true; }

    void writeServo(int channel, float position_0_to_1) override {
        if (channel < 0 || channel > 2) {
            return;
        }

        ActuatorCommand cmd = last_cmd_;
        const float symm = position_0_to_1 * 2.0f - 1.0f;
        if (channel == 0) {
            cmd.left_elevon = symm;
        } else if (channel == 1) {
            cmd.right_elevon = symm;
        } else if (channel == 2) {
            cmd.rudder = symm;
        }

        last_cmd_ = cmd;
        board_.writeActuators(last_cmd_);
    }

    bool initMotors() override { return true; }

    void writeMotor(int channel, float throttle_0_to_1) override {
        if (channel != 0) {
            return;
        }

        last_cmd_.throttle = throttle_0_to_1;
        board_.writeActuators(last_cmd_);
    }

    void digitalWrite(int pin, bool high) override { ::digitalWrite(pin, high ? HIGH : LOW); }
    bool digitalRead(int pin) override { return ::digitalRead(pin) == HIGH; }
    void pinMode(int pin, int mode) override { ::pinMode(pin, mode == 0 ? INPUT : OUTPUT); }

    void setStatusLED(bool on) override { ::digitalWrite(LED_BUILTIN, on ? HIGH : LOW); }

    void blinkStatusLED(int count, int on_ms, int off_ms) override {
        for (int i = 0; i < count; ++i) {
            setStatusLED(true);
            ::delay(on_ms);
            setStatusLED(false);
            ::delay(off_ms);
        }
    }

private:
    TeensyHAL& board_;
    ActuatorCommand last_cmd_{};
};

TeensyHAL board_hal;
LegacyHalAdapter hal_adapter(board_hal);
FlightController* controller = nullptr;

}  // namespace

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    delay(50);

    controller = new FlightController(&hal_adapter);
    if (!controller || !controller->initialize()) {
        Serial.println("[Teensy] FlightController init failed");
        while (true) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(150);
            digitalWrite(LED_BUILTIN, LOW);
            delay(150);
        }
    }

    Serial.println("[Teensy] FlightController init OK");
}

void loop() {
    if (controller) {
        controller->update();
    }
}
