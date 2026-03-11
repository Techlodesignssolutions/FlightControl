#include "ScheduledLQR.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace {

float lerp(float a, float b, float t) {
    return a + (b - a) * t;
}

std::size_t skipWhitespace(const std::string& text, std::size_t pos) {
    while (pos < text.size() && std::isspace(static_cast<unsigned char>(text[pos])) != 0) {
        ++pos;
    }
    return pos;
}

bool findKey(const std::string& text, const std::string& key, std::size_t& key_pos) {
    const std::string token = "\"" + key + "\"";
    key_pos = text.find(token);
    return key_pos != std::string::npos;
}

bool findMatchingBracket(const std::string& text, std::size_t open_pos, char open_ch, char close_ch, std::size_t& close_pos) {
    int depth = 0;
    bool in_string = false;
    bool escaped = false;

    for (std::size_t i = open_pos; i < text.size(); ++i) {
        const char c = text[i];

        if (in_string) {
            if (escaped) {
                escaped = false;
            } else if (c == '\\') {
                escaped = true;
            } else if (c == '"') {
                in_string = false;
            }
            continue;
        }

        if (c == '"') {
            in_string = true;
            continue;
        }

        if (c == open_ch) {
            ++depth;
            continue;
        }
        if (c == close_ch) {
            --depth;
            if (depth == 0) {
                close_pos = i;
                return true;
            }
        }
    }

    return false;
}

bool extractStructuredValue(const std::string& source, const std::string& key, char open_ch, char close_ch, std::string& inner_value) {
    std::size_t key_pos = 0;
    if (!findKey(source, key, key_pos)) {
        return false;
    }

    std::size_t colon_pos = source.find(':', key_pos);
    if (colon_pos == std::string::npos) {
        return false;
    }

    std::size_t value_pos = skipWhitespace(source, colon_pos + 1);
    if (value_pos >= source.size() || source[value_pos] != open_ch) {
        return false;
    }

    std::size_t close_pos = 0;
    if (!findMatchingBracket(source, value_pos, open_ch, close_ch, close_pos)) {
        return false;
    }

    inner_value = source.substr(value_pos + 1, close_pos - value_pos - 1);
    return true;
}

bool extractRawScalarToken(const std::string& source, const std::string& key, std::string& token) {
    std::size_t key_pos = 0;
    if (!findKey(source, key, key_pos)) {
        return false;
    }

    std::size_t colon_pos = source.find(':', key_pos);
    if (colon_pos == std::string::npos) {
        return false;
    }

    std::size_t begin = skipWhitespace(source, colon_pos + 1);
    if (begin >= source.size()) {
        return false;
    }

    if (source[begin] == '"') {
        std::size_t end_quote = begin + 1;
        bool escaped = false;
        while (end_quote < source.size()) {
            const char c = source[end_quote];
            if (escaped) {
                escaped = false;
            } else if (c == '\\') {
                escaped = true;
            } else if (c == '"') {
                break;
            }
            ++end_quote;
        }
        if (end_quote >= source.size() || source[end_quote] != '"') {
            return false;
        }

        token = source.substr(begin, (end_quote - begin + 1));
        return true;
    }

    std::size_t end = begin;
    while (end < source.size()) {
        const char c = source[end];
        if (c == ',' || c == '}' || c == ']' || std::isspace(static_cast<unsigned char>(c)) != 0) {
            break;
        }
        ++end;
    }

    token = source.substr(begin, end - begin);
    return !token.empty();
}

bool parseNumber(const std::string& token, float& out_value) {
    char* end_ptr = nullptr;
    out_value = std::strtof(token.c_str(), &end_ptr);
    if (end_ptr == token.c_str()) {
        return false;
    }

    while (*end_ptr != '\0') {
        if (std::isspace(static_cast<unsigned char>(*end_ptr)) == 0) {
            return false;
        }
        ++end_ptr;
    }
    return std::isfinite(out_value) != 0;
}

bool parseNumberArray(const std::string& array_body, std::vector<float>& out) {
    out.clear();
    std::size_t pos = 0;

    while (pos < array_body.size()) {
        pos = skipWhitespace(array_body, pos);
        if (pos >= array_body.size()) {
            break;
        }

        std::size_t next_comma = array_body.find(',', pos);
        std::string token = (next_comma == std::string::npos)
            ? array_body.substr(pos)
            : array_body.substr(pos, next_comma - pos);

        float value = 0.0f;
        if (!parseNumber(token, value)) {
            return false;
        }
        out.push_back(value);

        if (next_comma == std::string::npos) {
            break;
        }
        pos = next_comma + 1;
    }

    return !out.empty();
}

bool extractNumber(const std::string& source, const std::string& key, float& out_value) {
    std::string token;
    if (!extractRawScalarToken(source, key, token)) {
        return false;
    }
    return parseNumber(token, out_value);
}

bool extractString(const std::string& source, const std::string& key, std::string& out_value) {
    std::string token;
    if (!extractRawScalarToken(source, key, token)) {
        return false;
    }

    if (token.size() < 2 || token.front() != '"' || token.back() != '"') {
        return false;
    }

    out_value = token.substr(1, token.size() - 2);
    return !out_value.empty();
}

bool extractNumberArray(const std::string& source, const std::string& key, std::vector<float>& out_values) {
    std::string array_body;
    if (!extractStructuredValue(source, key, '[', ']', array_body)) {
        return false;
    }
    return parseNumberArray(array_body, out_values);
}

}  // namespace

bool ScheduledLQR::isAbsolutePath(const std::string& path) {
    if (path.size() >= 3 && std::isalpha(static_cast<unsigned char>(path[0])) != 0 && path[1] == ':' &&
        (path[2] == '\\' || path[2] == '/')) {
        return true;
    }
    if (path.size() >= 2 && path[0] == '\\' && path[1] == '\\') {
        return true;
    }
    if (!path.empty() && path[0] == '/') {
        return true;
    }
    return false;
}

float ScheduledLQR::clamp(float value, float lo, float hi) {
    return std::max(lo, std::min(hi, value));
}

bool ScheduledLQR::loadFromJsonFile(const char* absolute_path, ScheduleConfig& out_config, std::string* error_message) const {
    auto fail = [&](const char* msg) {
        if (error_message != nullptr) {
            *error_message = msg;
        }
        return false;
    };

    if (absolute_path == nullptr) {
        return fail("lqr_schedule_path is null");
    }

    const std::string path(absolute_path);
    if (!isAbsolutePath(path)) {
        return fail("lqr_schedule_path must be absolute");
    }

    std::ifstream input(path);
    if (!input.good()) {
        return fail("unable to open LQR schedule JSON file");
    }

    std::ostringstream oss;
    oss << input.rdbuf();
    const std::string json = oss.str();

    if (json.empty()) {
        return fail("LQR schedule JSON file is empty");
    }

    out_config = ScheduleConfig{};

    std::string metadata_object;
    if (!extractStructuredValue(json, "metadata", '{', '}', metadata_object)) {
        return fail("missing required metadata object");
    }

    if (!extractString(metadata_object, "name", out_config.name)) {
        return fail("missing required metadata.name string");
    }

    float version = 0.0f;
    if (!extractNumber(json, "version", version)) {
        return fail("missing required version");
    }
    out_config.version = static_cast<int>(version);
    if (out_config.version <= 0) {
        return fail("version must be >= 1");
    }

    std::vector<float> breakpoints;
    if (!extractNumberArray(json, "airspeed_breakpoints", breakpoints)) {
        return fail("missing required airspeed_breakpoints array");
    }

    if (breakpoints.size() < 2) {
        return fail("airspeed_breakpoints must contain at least two values");
    }

    for (std::size_t i = 1; i < breakpoints.size(); ++i) {
        if (!(breakpoints[i] > breakpoints[i - 1])) {
            return fail("airspeed_breakpoints must be strictly increasing");
        }
    }

    std::string roll_object;
    std::string pitch_object;
    std::string yaw_object;
    if (!extractStructuredValue(json, "roll", '{', '}', roll_object)) {
        return fail("missing required roll object");
    }
    if (!extractStructuredValue(json, "pitch", '{', '}', pitch_object)) {
        return fail("missing required pitch object");
    }
    if (!extractStructuredValue(json, "yaw", '{', '}', yaw_object)) {
        return fail("missing required yaw object");
    }

    std::vector<float> roll_k_error;
    std::vector<float> roll_k_rate;
    std::vector<float> roll_trim;
    std::vector<float> pitch_k_error;
    std::vector<float> pitch_k_rate;
    std::vector<float> pitch_trim;
    std::vector<float> yaw_k_error;
    std::vector<float> yaw_k_rate;
    std::vector<float> yaw_trim;
    std::vector<float> yaw_coord_gain;

    if (!extractNumberArray(roll_object, "k_error", roll_k_error) ||
        !extractNumberArray(roll_object, "k_rate", roll_k_rate) ||
        !extractNumberArray(roll_object, "trim", roll_trim)) {
        return fail("roll object must contain k_error, k_rate, trim arrays");
    }

    if (!extractNumberArray(pitch_object, "k_error", pitch_k_error) ||
        !extractNumberArray(pitch_object, "k_rate", pitch_k_rate) ||
        !extractNumberArray(pitch_object, "trim", pitch_trim)) {
        return fail("pitch object must contain k_error, k_rate, trim arrays");
    }

    if (!extractNumberArray(yaw_object, "k_error", yaw_k_error) ||
        !extractNumberArray(yaw_object, "k_rate", yaw_k_rate) ||
        !extractNumberArray(yaw_object, "trim", yaw_trim) ||
        !extractNumberArray(yaw_object, "coordination_gain", yaw_coord_gain)) {
        return fail("yaw object must contain k_error, k_rate, trim, coordination_gain arrays");
    }

    const std::size_t n = breakpoints.size();
    auto same_length = [n](const std::vector<float>& v) { return v.size() == n; };

    if (!same_length(roll_k_error) || !same_length(roll_k_rate) || !same_length(roll_trim) ||
        !same_length(pitch_k_error) || !same_length(pitch_k_rate) || !same_length(pitch_trim) ||
        !same_length(yaw_k_error) || !same_length(yaw_k_rate) || !same_length(yaw_trim) ||
        !same_length(yaw_coord_gain)) {
        return fail("all per-axis arrays must match airspeed_breakpoints length");
    }

    std::string adaptive_object;
    if (!extractStructuredValue(json, "adaptive", '{', '}', adaptive_object)) {
        return fail("missing required adaptive object");
    }

    if (!extractNumber(adaptive_object, "learning_rate", out_config.adaptive.learning_rate) ||
        !extractNumber(adaptive_object, "leakage", out_config.adaptive.leakage) ||
        !extractNumber(adaptive_object, "weight_limit", out_config.adaptive.weight_limit) ||
        !extractNumber(adaptive_object, "output_limit_roll", out_config.adaptive.output_limit_roll) ||
        !extractNumber(adaptive_object, "output_limit_pitch", out_config.adaptive.output_limit_pitch) ||
        !extractNumber(adaptive_object, "output_limit_yaw", out_config.adaptive.output_limit_yaw) ||
        !extractNumber(adaptive_object, "near_stall_airspeed", out_config.adaptive.near_stall_airspeed) ||
        !extractNumber(adaptive_object, "mode_change_freeze_s", out_config.adaptive.mode_change_freeze_s) ||
        !extractNumber(adaptive_object, "stick_step_threshold", out_config.adaptive.stick_step_threshold) ||
        !extractNumber(adaptive_object, "regime_stability_delta", out_config.adaptive.regime_stability_delta)) {
        return fail("adaptive object is missing one or more required numeric fields");
    }

    std::string safety_object;
    if (!extractStructuredValue(json, "safety", '{', '}', safety_object)) {
        return fail("missing required safety object");
    }

    if (!extractNumber(safety_object, "adaptive_limit_roll", out_config.safety.adaptive_limit_roll) ||
        !extractNumber(safety_object, "adaptive_limit_pitch", out_config.safety.adaptive_limit_pitch) ||
        !extractNumber(safety_object, "adaptive_limit_yaw", out_config.safety.adaptive_limit_yaw) ||
        !extractNumber(safety_object, "adaptive_rate_limit_roll", out_config.safety.adaptive_rate_limit_roll) ||
        !extractNumber(safety_object, "adaptive_rate_limit_pitch", out_config.safety.adaptive_rate_limit_pitch) ||
        !extractNumber(safety_object, "adaptive_rate_limit_yaw", out_config.safety.adaptive_rate_limit_yaw) ||
        !extractNumber(safety_object, "total_limit_roll", out_config.safety.total_limit_roll) ||
        !extractNumber(safety_object, "total_limit_pitch", out_config.safety.total_limit_pitch) ||
        !extractNumber(safety_object, "total_limit_yaw", out_config.safety.total_limit_yaw)) {
        return fail("safety object is missing one or more required numeric fields");
    }

    std::string controller_object;
    if (!extractStructuredValue(json, "controller", '{', '}', controller_object)) {
        return fail("missing required controller object");
    }

    if (!extractNumber(controller_object, "yaw_stick_gain", out_config.controller.yaw_stick_gain) ||
        !extractNumber(controller_object, "max_roll_cmd_rad", out_config.controller.max_roll_cmd_rad) ||
        !extractNumber(controller_object, "max_pitch_cmd_rad", out_config.controller.max_pitch_cmd_rad)) {
        return fail("controller object is missing one or more required numeric fields");
    }

    if (out_config.controller.max_roll_cmd_rad <= 0.0f || out_config.controller.max_pitch_cmd_rad <= 0.0f) {
        return fail("controller max command fields must be > 0");
    }

    out_config.points.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        SchedulePoint point;
        point.airspeed = breakpoints[i];
        point.roll.k_error = roll_k_error[i];
        point.roll.k_rate = roll_k_rate[i];
        point.roll.trim = roll_trim[i];

        point.pitch.k_error = pitch_k_error[i];
        point.pitch.k_rate = pitch_k_rate[i];
        point.pitch.trim = pitch_trim[i];

        point.yaw.k_error = yaw_k_error[i];
        point.yaw.k_rate = yaw_k_rate[i];
        point.yaw.trim = yaw_trim[i];

        point.yaw_coord_gain = yaw_coord_gain[i];
        out_config.points[i] = point;
    }

    if (out_config.adaptive.learning_rate <= 0.0f || out_config.adaptive.weight_limit <= 0.0f) {
        return fail("adaptive learning_rate and weight_limit must be > 0");
    }

    if (out_config.adaptive.leakage < 0.0f) {
        return fail("adaptive leakage must be >= 0");
    }

    return true;
}

bool ScheduledLQR::configure(const ScheduleConfig& config, std::string* error_message) {
    auto fail = [&](const char* msg) {
        if (error_message != nullptr) {
            *error_message = msg;
        }
        configured_ = false;
        return false;
    };

    if (config.points.size() < 2) {
        return fail("schedule must contain at least two points");
    }

    for (std::size_t i = 1; i < config.points.size(); ++i) {
        if (!(config.points[i].airspeed > config.points[i - 1].airspeed)) {
            return fail("schedule airspeed points must be strictly increasing");
        }
    }

    config_ = config;
    configured_ = true;
    return true;
}

ScheduledLQR::InterpIndex ScheduledLQR::getInterpolation(float airspeed) const {
    InterpIndex idx;

    if (!configured_ || config_.points.empty()) {
        return idx;
    }

    if (airspeed <= config_.points.front().airspeed) {
        idx.low = 0;
        idx.high = 0;
        idx.t = 0.0f;
        return idx;
    }

    if (airspeed >= config_.points.back().airspeed) {
        idx.low = config_.points.size() - 1;
        idx.high = idx.low;
        idx.t = 0.0f;
        return idx;
    }

    for (std::size_t i = 0; i + 1 < config_.points.size(); ++i) {
        const float v0 = config_.points[i].airspeed;
        const float v1 = config_.points[i + 1].airspeed;

        if (airspeed >= v0 && airspeed <= v1) {
            idx.low = i;
            idx.high = i + 1;
            const float span = std::max(1e-6f, v1 - v0);
            idx.t = clamp((airspeed - v0) / span, 0.0f, 1.0f);
            return idx;
        }
    }

    idx.low = config_.points.size() - 1;
    idx.high = idx.low;
    idx.t = 0.0f;
    return idx;
}

float ScheduledLQR::interpolateAxis(float airspeed, AxisPoint SchedulePoint::*axis_member, float error, float rate) const {
    if (!configured_) {
        return 0.0f;
    }

    const InterpIndex idx = getInterpolation(airspeed);
    const AxisPoint& a0 = config_.points[idx.low].*axis_member;
    const AxisPoint& a1 = config_.points[idx.high].*axis_member;

    const float k_error = lerp(a0.k_error, a1.k_error, idx.t);
    const float k_rate = lerp(a0.k_rate, a1.k_rate, idx.t);
    const float trim = lerp(a0.trim, a1.trim, idx.t);

    return trim + k_error * error - k_rate * rate;
}

float ScheduledLQR::computeRoll(float airspeed, float phi, float p, float phi_cmd) const {
    const float e_phi = phi_cmd - phi;
    return interpolateAxis(airspeed, &SchedulePoint::roll, e_phi, p);
}

float ScheduledLQR::computePitch(float airspeed, float theta, float q, float theta_cmd) const {
    const float e_theta = theta_cmd - theta;
    return interpolateAxis(airspeed, &SchedulePoint::pitch, e_theta, q);
}

float ScheduledLQR::computeYaw(float airspeed, float r, float r_cmd) const {
    const float e_r = r_cmd - r;
    return interpolateAxis(airspeed, &SchedulePoint::yaw, e_r, r);
}

float ScheduledLQR::interpolateYawCoordinationGain(float airspeed) const {
    if (!configured_) {
        return 0.0f;
    }

    const InterpIndex idx = getInterpolation(airspeed);
    return lerp(config_.points[idx.low].yaw_coord_gain, config_.points[idx.high].yaw_coord_gain, idx.t);
}

