#include <iostream>
#include <iomanip>
#include <atomic>
#include <cmath>
#include <vector>
#define NOMINMAX
#include <algorithm>
#include <chrono>
#include <cstring>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <mstcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>
#define SOCKET int
#define INVALID_SOCKET (-1)
#define SOCKET_ERROR   (-1)
#define closesocket close
#ifdef __linux__
#include <linux/if_ether.h>
#endif
#endif

#define MINIAUDIO_IMPLEMENTATION
#include "miniaudio.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#pragma pack(push, 1)
struct IPv4Header {
    uint8_t version_ihl;
    uint8_t tos;
    uint16_t total_length;
    uint16_t id;
    uint16_t flags_offset;
    uint8_t ttl;
    uint8_t protocol;
    uint16_t checksum;
    uint32_t src_ip;
    uint32_t dst_ip;
};
#pragma pack(pop)

constexpr int SAMPLE_RATE = 44100;
constexpr int MAX_VOICES = 24;
constexpr float TWO_PI = 2.0f * static_cast<float>(M_PI);
constexpr int QUEUE_SIZE = 512;

constexpr float SCALE[] = {
    54.00f, 64.00f, 72.00f, 81.00f, 96.00f,
    108.00f, 128.00f, 144.00f, 162.00f, 192.00f,
    216.00f, 256.00f, 288.00f, 324.00f, 384.00f,
    432.00f, 512.00f, 576.00f,
};
constexpr int SCALE_SIZE = 18;

enum VoiceType {
    SINE_PAD = 0,
    SOFT_FM,
    SMOOTH_PLUCK,
    FLUTE_PAD,
    GLASS_PAD,
    STRING_PAD,
    CHOIR_PAD,
    SHIMMER,
    NUM_TYPES
};

struct Voice {
    float freq = 0;
    float phase = 0;
    float phase2 = 0;
    float phase3 = 0;
    float phase4 = 0;
    float phase5 = 0;
    float phaseMod = 0;
    float phaseMod2 = 0;
    float amp = 0;
    float env = 0;
    int env_state = 0;

    float attack_rate = 0;
    float decay_rate = 0;

    float panPos = 0.5f;
    float panTarget = 0.5f;
    float panSpeed = 0.0002f;

    float vibPhase = 0;
    float amPhase = 0;

    VoiceType type = SINE_PAD;

    std::vector<float> ks_buf;
    int ks_idx = 0;
    float ks_lp = 0;

    float fm_ratio = 2.0f;
    float fm_depth = 0.5f;
    float fm_ratio2 = 3.0f;
    float fm_depth2 = 0.1f;

    float detune = 1.0f;
    float detune2 = 1.0f;
    float detune3 = 1.0f;
};

struct NoteEvent {
    float freq;
    float vol;
    VoiceType type;
    float panTarget;
};

NoteEvent note_queue[QUEUE_SIZE];
std::atomic<int> q_head{0}, q_tail{0};

Voice voices[MAX_VOICES];

struct CombFilter {
    std::vector<float> buf;
    int idx = 0;
    float feedback = 0;
    float damp = 0;
    float store = 0;

    void init(int delayLen, float fb, float dp) {
        buf.assign(delayLen, 0.0f);
        feedback = fb;
        damp = dp;
    }

    float process(float in) {
        float out = buf[idx];
        store = out * (1.0f - damp) + store * damp;
        buf[idx] = in + store * feedback;
        idx = (idx + 1) % (int) buf.size();
        return out;
    }
};

struct AllpassFilter {
    std::vector<float> buf;
    int idx = 0;
    float feedback = 0;

    void init(int delayLen, float fb) {
        buf.assign(delayLen, 0.0f);
        feedback = fb;
    }

    float process(float in) {
        float bo = buf[idx];
        buf[idx] = in + bo * feedback;
        idx = (idx + 1) % (int) buf.size();
        return -in + bo;
    }
};

struct Reverb {
    CombFilter combL[6], combR[6];
    AllpassFilter apL[3], apR[3];

    std::vector<float> preL, preR;
    int preIdx = 0;

    void init() {
        int pre = (int) (0.035f * SAMPLE_RATE);
        preL.assign(pre, 0.0f);
        preR.assign(pre, 0.0f);

        const int d[6] = {1867, 1993, 1747, 1621, 1453, 1559};
        const int stereoSpread = 31;

        for (int i = 0; i < 6; i++) {
            combL[i].init(d[i], 0.90f, 0.35f);
            combR[i].init(d[i] + stereoSpread, 0.90f, 0.35f);
        }

        apL[0].init(701, 0.5f);
        apR[0].init(732, 0.5f);
        apL[1].init(527, 0.5f);
        apR[1].init(558, 0.5f);
        apL[2].init(389, 0.5f);
        apR[2].init(420, 0.5f);
    }

    void process(float inL, float inR, float &outL, float &outR) {
        float pdL = preL[preIdx];
        float pdR = preR[preIdx];
        preL[preIdx] = inL;
        preR[preIdx] = inR;
        preIdx = (preIdx + 1) % (int) preL.size();

        float mono = (pdL + pdR) * 0.5f * 0.010f;

        float sL = 0, sR = 0;
        for (int i = 0; i < 6; i++) {
            sL += combL[i].process(mono);
            sR += combR[i].process(mono);
        }

        for (int i = 0; i < 3; i++) {
            sL = apL[i].process(sL);
            sR = apR[i].process(sR);
        }

        outL = sL;
        outR = sR;
    }
} reverb;

struct LP1S {
    float sL = 0, sR = 0;

    void process(float &L, float &R, float c) {
        sL += (L - sL) * c;
        sR += (R - sR) * c;
        L = sL;
        R = sR;
    }
} reverbLP;

struct DCBlock {
    float xm1 = 0, ym1 = 0;

    float process(float x) {
        float y = x - xm1 + 0.995f * ym1;
        xm1 = x;
        ym1 = y;
        return y;
    }
} dcL, dcR;

float global_time = 0;

int find_voice() {
    for (int i = 0; i < MAX_VOICES; i++)
        if (voices[i].env_state == 0) return i;

    int best = 0;
    float mn = 1e9f;
    for (int i = 0; i < MAX_VOICES; i++) {
        if (voices[i].env < mn) {
            mn = voices[i].env;
            best = i;
        }
    }
    return best;
}

void audio_data_callback(ma_device *, void *pOutput, const void *, ma_uint32 frameCount) {
    auto *fOut = static_cast<float *>(pOutput);

    while (q_tail.load() != q_head.load()) {
        NoteEvent ev = note_queue[q_tail.load()];
        q_tail.store((q_tail.load() + 1) % QUEUE_SIZE);

        int vi = find_voice();
        Voice &v = voices[vi];

        v.freq = ev.freq;
        v.amp = ev.vol;
        v.type = ev.type;
        v.env_state = 1;
        v.env = 0.0f;
        v.phase = 0;
        v.phase2 = (float) vi * 0.30f;
        v.phase3 = (float) vi * 0.55f;
        v.phase4 = (float) vi * 0.80f;
        v.phase5 = (float) vi * 0.15f;
        v.phaseMod = 0;
        v.phaseMod2 = 0;
        v.amPhase = (float) vi * 0.9f;
        v.vibPhase = (float) vi * 0.50f;

        v.panPos = (vi & 1) ? 0.35f : 0.65f;
        v.panTarget = ev.panTarget;
        v.panSpeed = 0.00015f + (vi % 5) * 0.00004f;

        v.detune = 1.0f + ((vi % 7) - 3) * 0.0007f;
        v.detune2 = 1.0f + ((vi % 5) - 2) * 0.0011f;
        v.detune3 = 1.0f + ((vi % 9) - 4) * 0.0005f;

        switch (ev.type) {
            case SINE_PAD:
                v.attack_rate = 1.0f / (SAMPLE_RATE * 0.25f);
                v.decay_rate = 0.999970f;
                break;

            case SOFT_FM:
                v.attack_rate = 1.0f / (SAMPLE_RATE * 0.18f);
                v.decay_rate = 0.999955f;
                v.fm_ratio = 2.0f + (vi % 3) * 0.01f;
                v.fm_depth = 0.4f + (vi % 4) * 0.08f;
                break;

            case SMOOTH_PLUCK: {
                v.attack_rate = 1.0f / (SAMPLE_RATE * 0.005f);
                v.decay_rate = 0.999940f;

                int ks_len = (int) (SAMPLE_RATE / ev.freq) + 1;
                v.ks_buf.assign(ks_len, 0.0f);
                for (int k = 0; k < ks_len; k++)
                    v.ks_buf[k] = std::sin(TWO_PI * k / ks_len);
                v.ks_idx = 0;
                v.ks_lp = 0;
                break;
            }

            case FLUTE_PAD:
                v.attack_rate = 1.0f / (SAMPLE_RATE * 0.30f);
                v.decay_rate = 0.999975f;
                break;

            case GLASS_PAD:
                v.attack_rate = 1.0f / (SAMPLE_RATE * 0.40f);
                v.decay_rate = 0.999985f;
                v.fm_ratio = 3.0f + (vi % 3) * 0.005f;
                v.fm_depth = 0.15f;
                break;

            case STRING_PAD:
                v.attack_rate = 1.0f / (SAMPLE_RATE * 1.20f);
                v.decay_rate = 0.999992f;
                break;

            case CHOIR_PAD:
                v.attack_rate = 1.0f / (SAMPLE_RATE * 0.55f);
                v.decay_rate = 0.999978f;
                break;

            case SHIMMER:
                v.attack_rate = 1.0f / (SAMPLE_RATE * 0.70f);
                v.decay_rate = 0.999980f;
                v.fm_ratio = 2.0f;
                v.fm_depth = 0.06f;
                break;

            default: break;
        }
    }

    float globalLFO = 0.92f + 0.08f * std::sin(global_time * 0.12f);

    for (ma_uint32 i = 0; i < frameCount; i++) {
        global_time += 1.0f / SAMPLE_RATE;
        float mix_L = 0, mix_R = 0;

        for (auto &v: voices) {
            if (v.env_state == 0) continue;

            if (v.env_state == 1) {
                v.env += v.attack_rate;
                if (v.env >= 1.0f) {
                    v.env = 1.0f;
                    v.env_state = 2;
                }
            } else {
                v.env *= v.decay_rate;
                if (v.env < 0.0003f) {
                    v.env = 0;
                    v.env_state = 0;
                    continue;
                }
            }

            v.vibPhase += TWO_PI * 3.5f / SAMPLE_RATE;
            float vib = 1.0f + 0.0018f * std::sin(v.vibPhase);

            v.panPos += (v.panTarget - v.panPos) * v.panSpeed;
            float ang = v.panPos * (TWO_PI / 4.0f);
            float panL = std::cos(ang);
            float panR = std::sin(ang);

            float freq = v.freq * vib;
            float dphi = TWO_PI * freq / SAMPLE_RATE;
            float sample = 0;

            switch (v.type) {
                case SINE_PAD:
                    sample = std::sin(v.phase3) * 0.30f
                             + std::sin(v.phase) * 0.55f
                             + std::sin(v.phase2) * 0.22f;
                    v.phase += dphi;
                    v.phase2 += dphi * v.detune;
                    v.phase3 += dphi * 0.5f;
                    break;

                case SOFT_FM: {
                    float mod = v.fm_depth * v.env * std::sin(v.phaseMod);
                    sample = std::sin(v.phase + mod) * 0.75f
                             + std::sin(v.phase) * 0.25f;
                    v.phase += dphi;
                    v.phaseMod += dphi * v.fm_ratio;
                    v.phase2 += dphi * v.detune;
                    break;
                }

                case SMOOTH_PLUCK:
                    if (!v.ks_buf.empty()) {
                        int len = (int) v.ks_buf.size();
                        sample = v.ks_buf[v.ks_idx];
                        int nx = (v.ks_idx + 1) % len;
                        float avg = (v.ks_buf[v.ks_idx] + v.ks_buf[nx]) * 0.4992f;
                        v.ks_lp += (avg - v.ks_lp) * 0.7f;
                        v.ks_buf[v.ks_idx] = v.ks_lp;
                        v.ks_idx = nx;
                    }
                    break;

                case FLUTE_PAD: {
                    float s1 = std::sin(v.phase);
                    float shaped = std::sin(s1 * 1.6f);
                    sample = shaped * 0.62f
                             + std::sin(v.phase2) * 0.28f
                             + std::sin(v.phase3) * 0.18f;
                    v.phase += dphi;
                    v.phase2 += dphi * v.detune;
                    v.phase3 += dphi * 0.5f;
                    break;
                }

                case GLASS_PAD: {
                    float h1 = std::sin(v.phase);
                    float h3 = std::sin(v.phase2) * 0.38f;
                    float h5 = std::sin(v.phase3) * 0.14f;
                    float fmG = std::sin(v.phaseMod) * v.fm_depth * v.env;
                    float ring = std::sin(v.phase + fmG);
                    sample = h1 * 0.50f + ring * 0.20f + h3 + h5;
                    v.phase += dphi;
                    v.phase2 += dphi * 3.0f * v.detune;
                    v.phase3 += dphi * 5.0f * v.detune2;
                    v.phaseMod += dphi * v.fm_ratio;
                    break;
                }

                case STRING_PAD: {
                    float d1 = 1.0f + v.detune * 0.0015f;
                    float d2 = 1.0f - v.detune * 0.0012f;
                    float d3 = 1.0f + v.detune2 * 0.0022f;
                    float d4 = 1.0f - v.detune2 * 0.0008f;
                    sample = std::sin(v.phase) * 0.28f
                             + std::sin(v.phase2) * 0.28f
                             + std::sin(v.phase3) * 0.22f
                             + std::sin(v.phase4) * 0.18f
                             + std::sin(v.phase5) * 0.14f;
                    sample += std::sin(v.phase * 2.0f) * 0.06f;
                    v.phase += dphi;
                    v.phase2 += dphi * d1;
                    v.phase3 += dphi * d2;
                    v.phase4 += dphi * d3;
                    v.phase5 += dphi * d4;
                    break;
                }

                case CHOIR_PAD: {
                    v.amPhase += TWO_PI * 5.2f / SAMPLE_RATE;
                    float amEnv = 0.82f + 0.18f * std::sin(v.amPhase);
                    float voice1 = std::sin(v.phase) * amEnv;
                    float voice2 = std::sin(v.phase2) * (1.0f - amEnv * 0.15f);
                    float voice3 = std::sin(v.phase3) * (0.85f + amEnv * 0.08f);
                    float h2 = std::sin(v.phase * 2.0f + 0.3f) * 0.12f;
                    sample = voice1 * 0.42f + voice2 * 0.38f + voice3 * 0.28f + h2;
                    v.phase += dphi;
                    v.phase2 += dphi * v.detune;
                    v.phase3 += dphi * v.detune2;
                    break;
                }

                case SHIMMER: {
                    float shimVib = 1.0f + 0.0025f * std::sin(v.vibPhase * 0.6f);
                    float mod = std::sin(v.phaseMod) * v.fm_depth;
                    float fund = std::sin(v.phase + mod) * 0.55f;
                    float oct = std::sin(v.phase2 + mod * 0.5f) * 0.30f;
                    float fifth = std::sin(v.phase3) * 0.18f;
                    sample = fund + oct * shimVib + fifth;
                    v.phase += dphi;
                    v.phase2 += dphi * 2.0f * v.detune;
                    v.phase3 += dphi * 1.5f * v.detune2;
                    v.phaseMod += dphi * v.fm_ratio;
                    break;
                }

                default: break;
            }

            auto wrap = [](float &p) { if (p > TWO_PI) p -= TWO_PI; };
            wrap(v.phase);
            wrap(v.phase2);
            wrap(v.phase3);
            wrap(v.phase4);
            wrap(v.phase5);
            wrap(v.phaseMod);
            wrap(v.phaseMod2);

            float out = sample * v.amp * v.env * globalLFO;
            mix_L += out * panL;
            mix_R += out * panR;
        }

        float revL, revR;
        reverb.process(mix_L, mix_R, revL, revR);

        reverbLP.process(revL, revR, 0.08f);

        float wet = 0.65f;
        float outL = mix_L * (1.0f - wet * 0.3f) + revL * wet;
        float outR = mix_R * (1.0f - wet * 0.3f) + revR * wet;

        outL = std::tanh(outL * 1.05f) / 1.05f;
        outR = std::tanh(outR * 1.05f) / 1.05f;

        outL = dcL.process(outL);
        outR = dcR.process(outR);

        fOut[i * 2 + 0] = outL;
        fOut[i * 2 + 1] = outR;
    }
}

void send_to_audio(float freq, float vol, VoiceType type, float panTarget) {
    int head = q_head.load();
    int next = (head + 1) % QUEUE_SIZE;
    if (next != q_tail.load()) {
        note_queue[head] = {freq, vol, type, panTarget};
        q_head.store(next);
    }
}

void process_raw_bytes(const uint8_t *buffer, int size) {
    int ip_offset = 0;
#ifdef __linux__
    ip_offset = 14;
#endif

    if (size < ip_offset + (int) sizeof(IPv4Header)) return;

    const auto *ip = reinterpret_cast<const IPv4Header *>(buffer + ip_offset);

    if ((ip->version_ihl >> 4) != 4) return;

    if (ip->protocol != 6 && ip->protocol != 17) return;

    static auto last_note = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_note).count() < 110) return;
    last_note = now;

    const uint8_t *src = reinterpret_cast<const uint8_t *>(&ip->src_ip);
    const uint8_t *dst = reinterpret_cast<const uint8_t *>(&ip->dst_ip);
    const uint16_t pkt_len = ntohs(ip->total_length);
    const uint8_t tos_val = ip->tos;
    const uint8_t ttl_val = ip->ttl;

    int band = (pkt_len > 900) ? 0 : (pkt_len > 300) ? 1 : 2;
    int idx = band * 5 + (src[3] % 6);
    if (idx >= SCALE_SIZE) idx = SCALE_SIZE - 1;
    float freq = SCALE[idx];

    float norm = std::min((float) pkt_len / 1500.0f, 1.0f);
    float vol = 0.10f + (std::log1p(norm * 4.0f) / std::log1p(4.0f)) * 0.10f;

    float panTarget = (dst[2] % 101) / 100.0f;

    VoiceType vtype;
    bool isUDP = (ip->protocol == 17);
    bool hasDSCP = (tos_val >> 2) != 0;

    if (pkt_len < 150 && isUDP) vtype = SHIMMER;
    else if (pkt_len < 150) vtype = SMOOTH_PLUCK;
    else if (isUDP && pkt_len > 600) vtype = FLUTE_PAD;
    else if (isUDP && pkt_len <= 300) vtype = GLASS_PAD;
    else if (pkt_len > 900 && ttl_val <= 64) vtype = STRING_PAD;
    else if (ttl_val > 100) vtype = SOFT_FM;
    else if (src[3] % 2 == 0 && pkt_len > 150) vtype = CHOIR_PAD;
    else if (hasDSCP) vtype = SHIMMER;
    else vtype = SINE_PAD;

    send_to_audio(freq, vol, vtype, panTarget);

    const char *names[] = {"Pad  ", "FMPad", "Pluck", "Flute", "Glass", "Strng", "Bell ", "Choir", "Shmr "};
    const char *colors[] = {
        "\033[34m",
        "\033[33m",
        "\033[32m",
        "\033[36m",
        "\033[97m",
        "\033[35m",
        "\033[93m",
        "\033[95m",
        "\033[96m",
    };

    int bars = (int) (vol * 120);
    std::cout << colors[(int) vtype]
            << std::left << std::setw(6) << names[(int) vtype]
            << std::right << std::setw(5) << (int) freq << "Hz  "
            << std::string(bars, '\xB7')
            << "\033[0m\n";
}

int main() {
#ifdef _WIN32
    HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD dwMode = 0;
    GetConsoleMode(hOut, &dwMode);
    SetConsoleMode(hOut, dwMode | ENABLE_VIRTUAL_TERMINAL_PROCESSING);
#endif

    std::cout << "\033[34m  ～  NetSymphony  ～\n\033[0m\n";
    std::cout << "  Voices: Pad · FMPad · Pluck · Flute · Glass · String · Bell · Choir · Shimmer\n\n";

#ifdef _WIN32
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif

    SOCKET sniffer;

#ifdef _WIN32
    sniffer = socket(AF_INET, SOCK_RAW, IPPROTO_IP);
    if (sniffer == INVALID_SOCKET) {
        std::cerr << "Run as Administrator!\n";
        return 1;
    }

    char hostname[256];
    gethostname(hostname, sizeof(hostname));
    hostent *local = gethostbyname(hostname);
    if (!local) {
        std::cerr << "Cannot resolve hostname.\n";
        return 1;
    }

    std::cout << "=== Network Interfaces ===\n";
    int num_ifaces = 0;
    while (local->h_addr_list[num_ifaces]) {
        in_addr addr{};
        addr.s_addr = *reinterpret_cast<u_long *>(local->h_addr_list[num_ifaces]);
        std::cout << "[" << num_ifaces << "] " << inet_ntoa(addr) << "\n";
        num_ifaces++;
    }

    int choice = 0;
    if (num_ifaces > 1) {
        std::cout << "-> Interface: ";
        std::cin >> choice;
        if (choice < 0 || choice >= num_ifaces) choice = 0;
    }

    sockaddr_in dest{};
    dest.sin_family = AF_INET;
    dest.sin_addr.s_addr = *reinterpret_cast<u_long *>(local->h_addr_list[choice]);
    bind(sniffer, reinterpret_cast<struct sockaddr *>(&dest), sizeof(dest));

    int j = 1;
    DWORD bytesRet;
    if (WSAIoctl(sniffer, SIO_RCVALL, &j, sizeof(j), nullptr, 0, &bytesRet, nullptr, nullptr) == SOCKET_ERROR) {
        std::cerr << "Need Administrator rights.\n";
        return 1;
    }

#elif defined(__linux__)
    sniffer = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));

#else
    sniffer = socket(AF_INET, SOCK_RAW, IPPROTO_TCP);
#endif

    if (sniffer == INVALID_SOCKET) {
        std::cerr << "Failed to open raw socket.\n";
        return 1;
    }

    reverb.init();

    ma_device_config cfg = ma_device_config_init(ma_device_type_playback);
    cfg.playback.format = ma_format_f32;
    cfg.playback.channels = 2;
    cfg.sampleRate = SAMPLE_RATE;
    cfg.dataCallback = audio_data_callback;

    ma_device device;
    if (ma_device_init(nullptr, &cfg, &device) != MA_SUCCESS) {
        std::cerr << "Audio init failed.\n";
        return 1;
    }
    ma_device_start(&device);

    std::cout << "\033[34m  listening  ·  breathe  ·\033[0m\n\n";

    uint8_t buffer[65536];
    while (true) {
        int received = recvfrom(sniffer, reinterpret_cast<char *>(buffer), sizeof(buffer), 0, nullptr, nullptr);
        if (received > 0) process_raw_bytes(buffer, received);
    }

    ma_device_uninit(&device);
    closesocket(sniffer);
#ifdef _WIN32
    WSACleanup();
#endif
    return 0;
}
