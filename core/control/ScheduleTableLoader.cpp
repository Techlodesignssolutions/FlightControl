#include "ScheduleTableLoader.h"

#include <cctype>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace {

std::size_t skipWs(const std::string& s, std::size_t p) {
    while (p < s.size() && std::isspace(static_cast<unsigned char>(s[p])) != 0) {
        ++p;
    }
    return p;
}

bool keyPos(const std::string& s, const std::string& key, std::size_t& p) {
    p = s.find("\"" + key + "\"");
    return p != std::string::npos;
}

bool matchRange(const std::string& s, std::size_t open, char och, char cch, std::size_t& close) {
    int d = 0;
    for (std::size_t i = open; i < s.size(); ++i) {
        if (s[i] == och) {
            ++d;
        } else if (s[i] == cch) {
            --d;
            if (d == 0) {
                close = i;
                return true;
            }
        }
    }
    return false;
}

bool extractArrayBody(const std::string& s, const std::string& key, std::string& body) {
    std::size_t kp = 0;
    if (!keyPos(s, key, kp)) {
        return false;
    }
    const std::size_t c = s.find(':', kp);
    if (c == std::string::npos) {
        return false;
    }
    const std::size_t b = skipWs(s, c + 1);
    if (b >= s.size() || s[b] != '[') {
        return false;
    }
    std::size_t e = 0;
    if (!matchRange(s, b, '[', ']', e)) {
        return false;
    }
    body = s.substr(b + 1, e - b - 1);
    return true;
}

bool parseNum(const std::string& t, float& out) {
    char* end = nullptr;
    out = std::strtof(t.c_str(), &end);
    return end != t.c_str();
}

bool parseArrayNums(const std::string& body, std::vector<float>& out) {
    out.clear();
    std::size_t p = 0;
    while (p < body.size()) {
        p = skipWs(body, p);
        if (p >= body.size()) {
            break;
        }
        std::size_t c = body.find(',', p);
        std::string t = (c == std::string::npos) ? body.substr(p) : body.substr(p, c - p);
        float v = 0.0f;
        if (!parseNum(t, v)) {
            return false;
        }
        out.push_back(v);
        if (c == std::string::npos) {
            break;
        }
        p = c + 1;
    }
    return !out.empty();
}

bool extractNestedArrayNums(const std::string& root, const std::string& obj, const std::string& key, std::vector<float>& out) {
    std::size_t kp = 0;
    if (!keyPos(root, obj, kp)) {
        return false;
    }
    const std::size_t c = root.find(':', kp);
    if (c == std::string::npos) {
        return false;
    }
    std::size_t b = skipWs(root, c + 1);
    if (b >= root.size() || root[b] != '{') {
        return false;
    }
    std::size_t e = 0;
    if (!matchRange(root, b, '{', '}', e)) {
        return false;
    }
    const std::string body = root.substr(b + 1, e - b - 1);
    std::string arr;
    if (!extractArrayBody(body, key, arr)) {
        return false;
    }
    return parseArrayNums(arr, out);
}

}  // namespace

bool ScheduleTableLoader::loadFromJsonFile(const char* path, std::vector<LQRGainPoint>& out_points, std::string* error) const {
    auto fail = [&](const char* m) {
        if (error != nullptr) {
            *error = m;
        }
        return false;
    };

    if (path == nullptr) {
        return fail("null path");
    }

    std::ifstream in(path);
    if (!in.good()) {
        return fail("cannot open schedule json");
    }

    std::ostringstream ss;
    ss << in.rdbuf();
    const std::string json = ss.str();

    std::vector<float> v;
    std::vector<float> kphi;
    std::vector<float> kp;
    std::vector<float> ktheta;
    std::vector<float> kq;

    std::string b;
    if (!extractArrayBody(json, "airspeed_breakpoints", b) || !parseArrayNums(b, v)) {
        return fail("missing airspeed_breakpoints");
    }
    if (!extractNestedArrayNums(json, "roll", "k_phi", kphi) || !extractNestedArrayNums(json, "roll", "k_p", kp)) {
        return fail("missing roll gains");
    }
    if (!extractNestedArrayNums(json, "pitch", "k_theta", ktheta) || !extractNestedArrayNums(json, "pitch", "k_q", kq)) {
        return fail("missing pitch gains");
    }

    const std::size_t n = v.size();
    if (kphi.size() != n || kp.size() != n || ktheta.size() != n || kq.size() != n) {
        return fail("gain array lengths must match breakpoints");
    }

    out_points.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        out_points[i].airspeed_mps = v[i];
        out_points[i].k_phi = kphi[i];
        out_points[i].k_p = kp[i];
        out_points[i].k_theta = ktheta[i];
        out_points[i].k_q = kq[i];
    }
    return true;
}

bool ScheduleTableLoader::buildTable(std::vector<LQRGainPoint>& points, LQRScheduleTable& out_table) const {
    if (points.empty()) {
        return false;
    }
    out_table.points = points.data();
    out_table.count = points.size();
    return true;
}
