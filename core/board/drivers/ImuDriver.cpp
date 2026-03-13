#include "ImuDriver.h"

// NOTE: Stub transport implementation for bring-up; replace with real board I/O backend.

#include "../platform/SpeedyBeeF405WingPins.h"

bool ImuDriver::init() {
    initialized_ = platform_.initSpiBus(speedybee_f405_wing::IMU_SPI_BUS);
    return initialized_;
}

bool ImuDriver::read(ImuSample& out) const {
    if (!initialized_) {
        return false;
    }

    Stm32f4Platform::ImuRaw raw{};
    if (!platform_.readImuRaw(raw)) {
        return false;
    }

    out.gx_rad_s = raw.gx_rad_s;
    out.gy_rad_s = raw.gy_rad_s;
    out.gz_rad_s = raw.gz_rad_s;
    out.ax_m_s2 = raw.ax_m_s2;
    out.ay_m_s2 = raw.ay_m_s2;
    out.az_m_s2 = raw.az_m_s2;
    return true;
}

#if defined(UNIT_TEST) || defined(BENCH_HARNESS)
void ImuDriver::setSample(const ImuSample& sample) {
    Stm32f4Platform::ImuRaw raw{};
    raw.gx_rad_s = sample.gx_rad_s;
    raw.gy_rad_s = sample.gy_rad_s;
    raw.gz_rad_s = sample.gz_rad_s;
    raw.ax_m_s2 = sample.ax_m_s2;
    raw.ay_m_s2 = sample.ay_m_s2;
    raw.az_m_s2 = sample.az_m_s2;
    platform_.injectImuRaw(raw);
}
#endif
