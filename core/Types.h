#pragma once

#include <cmath>
#include <cstdint>

/**
 * Core data types used throughout the flight controller.
 */

struct Vector3 {
    float x;
    float y;
    float z;

    Vector3() : x(0), y(0), z(0) {}
    Vector3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vector3 operator+(const Vector3& other) const {
        return Vector3(x + other.x, y + other.y, z + other.z);
    }

    Vector3 operator-(const Vector3& other) const {
        return Vector3(x - other.x, y - other.y, z - other.z);
    }

    Vector3 operator*(float scalar) const {
        return Vector3(x * scalar, y * scalar, z * scalar);
    }

    float magnitude() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    void normalize() {
        float mag = magnitude();
        if (mag > 0.001f) {
            x /= mag;
            y /= mag;
            z /= mag;
        }
    }
};

struct Attitude {
    float roll;   // radians, positive = right wing down
    float pitch;  // radians, positive = nose up
    float yaw;    // radians, positive = nose right

    Attitude() : roll(0), pitch(0), yaw(0) {}
    Attitude(float r, float p, float y) : roll(r), pitch(p), yaw(y) {}
};

struct AngularRates {
    float roll_rate;   // rad/s, positive = rolling right
    float pitch_rate;  // rad/s, positive = pitching up
    float yaw_rate;    // rad/s, positive = yawing right

    AngularRates() : roll_rate(0), pitch_rate(0), yaw_rate(0) {}
    AngularRates(float r, float p, float y) : roll_rate(r), pitch_rate(p), yaw_rate(y) {}
};

struct RadioInputs {
    float throttle;  // 0..1
    float roll;      // -1..1
    float pitch;     // -1..1
    float yaw;       // -1..1
    float aux1;      // -1..1
    float aux2;      // -1..1
    bool armed;
    std::uint32_t last_update_time;  // milliseconds

    RadioInputs()
        : throttle(0.0f)
        , roll(0.0f)
        , pitch(0.0f)
        , yaw(0.0f)
        , aux1(0.0f)
        , aux2(0.0f)
        , armed(false)
        , last_update_time(0) {}
};

struct ControlSurfaces {
    float left_elevon;   // -1..1
    float right_elevon;  // -1..1
    float rudder;        // -1..1
    float throttle;      // 0..1

    ControlSurfaces() : left_elevon(0), right_elevon(0), rudder(0), throttle(0) {}
    ControlSurfaces(float l, float r, float rud, float thr)
        : left_elevon(l), right_elevon(r), rudder(rud), throttle(thr) {}
};

struct AircraftState {
    Attitude attitude;
    AngularRates rates;
    Vector3 acceleration;        // body-frame m/s^2 (gravity included)
    Vector3 linear_acceleration; // body-frame m/s^2 (gravity removed)
    Vector3 wind_disturbance;    // body-frame m/s^2 disturbance estimate
    float airspeed;              // m/s
    float altitude;              // m
    std::uint32_t timestamp;     // microseconds

    AircraftState() : airspeed(0), altitude(0), timestamp(0) {}
};

enum class ControlMode {
    MANUAL = 0,
    STABILIZE = 1,
    ALTITUDE = 2,
    POSITION = 3,
    AUTO = 4
};

struct PerformanceStats {
    float loop_time_avg_ms;
    float loop_time_max_ms;
    std::uint32_t loop_count;
    std::uint32_t overrun_count;
    float cpu_usage_percent;

    PerformanceStats()
        : loop_time_avg_ms(0.0f)
        , loop_time_max_ms(0.0f)
        , loop_count(0)
        , overrun_count(0)
        , cpu_usage_percent(0.0f) {}

    PerformanceStats(float avg, float max, std::uint32_t count, std::uint32_t overruns, float cpu)
        : loop_time_avg_ms(avg)
        , loop_time_max_ms(max)
        , loop_count(count)
        , overrun_count(overruns)
        , cpu_usage_percent(cpu) {}
};

struct DiagnosticInfo {
    bool imu_healthy;
    bool radio_healthy;
    bool airspeed_converged;
    bool adaptive_learning;
    std::uint32_t uptime_ms;

    DiagnosticInfo()
        : imu_healthy(false)
        , radio_healthy(false)
        , airspeed_converged(false)
        , adaptive_learning(false)
        , uptime_ms(0) {}
};
