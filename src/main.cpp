#include <iostream>
#include <iomanip>
#include <atomic>
#include <cmath>
#include <thread>
#include <vector>
#ifndef NOMINMAX
#define NOMINMAX
#endif
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
#include <ifaddrs.h>
#include <net/if.h>
#include <linux/if_packet.h>
#endif
#endif

#define MINIAUDIO_IMPLEMENTATION
#include "miniaudio.h"

#ifdef _WIN32
#ifndef SIO_RCVALL
#define SIO_RCVALL _WSAIOW(IOC_VENDOR,1)
#endif
#ifndef SIO_RCVALL_ON
#define SIO_RCVALL_ON 1
#endif
#ifndef SIO_RCVALL_OFF
#define SIO_RCVALL_OFF 0
#endif
#endif

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
constexpr int MAX_VOICES = 48;
constexpr float TWO_PI = 2.0f * static_cast<float>(M_PI);
constexpr int QUEUE_SIZE = 1024;

constexpr int TICK_INTERVAL = (int) (SAMPLE_RATE * 0.40f);
constexpr int MAX_NOTES_PER_CHORD = 6;

constexpr float SCALE[] = {
    65.41f,
    98.00f,
    130.81f,
    164.81f,
    196.00f,
    246.94f,
    293.66f,
    329.63f,
    369.99f,
    392.00f,
    493.88f,
    523.25f,
    587.33f,
    659.25f,
    739.99f,
    783.99f
};
constexpr int SCALE_SIZE = 16;

enum VoiceType {
    SINE_PAD = 0,
    SOFT_FM,
    WARM_PAD,
    WIND_PAD,
    CRYSTAL_PAD,
    CELLO_PAD,
    HALO_PAD,
    AMBIENT_CLOUD,
    DEEP_DRONE,
    CHIME_PAD,
    GHOST_CHOIR,
    LUNAR_DUST,
    NUM_TYPES
};

struct Delay3D {
    float buffer[128]{};
    int writeIdx = 0;

    void init() {
        std::fill(std::begin(buffer), std::end(buffer), 0.0f);
    }

    void write(const float sample) {
        buffer[writeIdx] = sample;
        writeIdx = (writeIdx + 1) % 128;
    }

    [[nodiscard]] float read(const float delaySamples) const {
        float readPos = static_cast<float>(writeIdx) - delaySamples;
        if (readPos < 0) readPos += 128.0f;
        const int idx1 = static_cast<int>(readPos);
        const int idx2 = (idx1 + 1) % 128;
        const float frac = readPos - static_cast<float>(idx1);
        return buffer[idx1] * (1.0f - frac) + buffer[idx2] * frac;
    }
};

struct Voice {
    float freq = 0, phase = 0, phase2 = 0, phase3 = 0, phase4 = 0;
    float phaseMod = 0, amp = 0, env = 0;
    int env_state = 0;
    float attack_rate = 0, decay_rate = 0;

    float angle = 0;
    float angleSpeed = 0;
    Delay3D delay3d;
    float lpL = 0, lpR = 0;

    float vibPhase = 0, amPhase = 0;
    VoiceType type = SINE_PAD;

    float fm_ratio = 2.0f, fm_depth = 0.1f;
    float detune = 1.0f, detune2 = 1.0f;
};

struct NoteEvent {
    float freq;
    float vol;
    VoiceType type;
    float angleStart;
};

NoteEvent note_queue[QUEUE_SIZE];
std::atomic<int> q_head{0}, q_tail{0};
Voice voices[MAX_VOICES];
int audio_sample_tick = 0;

struct CombFilter {
    std::vector<float> buf;
    int idx = 0;
    float feedback = 0, damp = 0, store = 0;

    void init(const int delayLen, const float fb, const float dp) {
        buf.assign(delayLen, 0.0f);
        feedback = fb;
        damp = dp;
    }

    float process(const float in) {
        const float out = buf[idx];
        store = out * (1.0f - damp) + store * damp;
        buf[idx] = in + store * feedback;
        idx = (idx + 1) % static_cast<int>(buf.size());
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

    float process(const float in) {
        const float bo = buf[idx];
        buf[idx] = in + bo * feedback;
        idx = (idx + 1) % static_cast<int>(buf.size());
        return -in + bo;
    }
};

struct Reverb {
    CombFilter combL[6], combR[6];
    AllpassFilter apL[3], apR[3];

    void init() {
        for (int i = 0; i < 6; i++) {
            const int d[6] = {1867, 1993, 1747, 1621, 1453, 1559};
            constexpr int stereoSpread = 37;
            combL[i].init(d[i], 0.95f, 0.40f);
            combR[i].init(d[i] + stereoSpread, 0.95f, 0.40f);
        }
        apL[0].init(701, 0.6f);
        apR[0].init(732, 0.6f);
        apL[1].init(527, 0.6f);
        apR[1].init(558, 0.6f);
        apL[2].init(389, 0.6f);
        apR[2].init(420, 0.6f);
    }

    void process(const float inL, const float inR, float &outL, float &outR) {
        float sL = 0, sR = 0;
        for (int i = 0; i < 6; i++) {
            sL += combL[i].process(inL * 0.010f);
            sR += combR[i].process(inR * 0.010f);
        }
        for (int i = 0; i < 3; i++) {
            sL = apL[i].process(sL);
            sR = apR[i].process(sR);
        }
        outL = sL;
        outR = sR;
    }
} reverb;

struct MasterLP {
    float sL = 0, sR = 0;

    void process(float &L, float &R, float c) {
        sL += (L - sL) * c;
        sR += (R - sR) * c;
        L = sL;
        R = sR;
    }
} filter;

struct DCBlock {
    float xm1 = 0, ym1 = 0;

    float process(float x) {
        float y = x - xm1 + 0.995f * ym1;
        xm1 = x;
        ym1 = y;
        return y;
    }
} dcL, dcR;

int find_voice() {
    for (int i = 0; i < MAX_VOICES; i++) if (voices[i].env_state == 0) return i;
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

    for (ma_uint32 i = 0; i < frameCount; i++) {
        audio_sample_tick++;
        if (audio_sample_tick >= TICK_INTERVAL) {
            audio_sample_tick = 0;
            int notes_triggered = 0;

            while (q_tail.load() != q_head.load() && notes_triggered < MAX_NOTES_PER_CHORD) {
                NoteEvent ev = note_queue[q_tail.load()];
                q_tail.store((q_tail.load() + 1) % QUEUE_SIZE);

                int vi = find_voice();

                auto &v = voices[vi];

                v.freq = ev.freq;
                v.amp = ev.vol;
                v.type = ev.type;
                v.env_state = 1;
                v.env = 0.0f;
                v.phase = 0;
                v.phase2 = vi * 0.3f;
                v.phase3 = vi * 0.7f;
                v.phase4 = vi * 0.5f;
                v.phaseMod = 0;
                v.amPhase = vi * 0.9f;
                v.vibPhase = vi * 0.5f;

                v.angle = ev.angleStart;
                v.angleSpeed = (vi % 2 == 0 ? 1.0f : -1.0f) * (TWO_PI / (SAMPLE_RATE * (10.0f + vi % 5)));

                v.detune = 1.0f + (vi % 7 - 3) * 0.0006f;
                v.detune2 = 1.0f + (vi % 5 - 2) * 0.0009f;
                v.delay3d.init();
                v.lpL = 0;
                v.lpR = 0;

                switch (ev.type) {
                    case SINE_PAD: v.attack_rate = 1.0f / (SAMPLE_RATE * 0.8f);
                        v.decay_rate = 0.999980f;
                        break;
                    case SOFT_FM: v.attack_rate = 1.0f / (SAMPLE_RATE * 1.0f);
                        v.decay_rate = 0.999985f;
                        v.fm_ratio = 2.0f;
                        v.fm_depth = 0.2f;
                        break;
                    case WARM_PAD: v.attack_rate = 1.0f / (SAMPLE_RATE * 1.2f);
                        v.decay_rate = 0.999988f;
                        break;
                    case WIND_PAD: v.attack_rate = 1.0f / (SAMPLE_RATE * 1.5f);
                        v.decay_rate = 0.999985f;
                        break;
                    case CRYSTAL_PAD: v.attack_rate = 1.0f / (SAMPLE_RATE * 0.8f);
                        v.decay_rate = 0.999990f;
                        v.fm_ratio = 3.0f;
                        v.fm_depth = 0.1f;
                        break;
                    case CELLO_PAD: v.attack_rate = 1.0f / (SAMPLE_RATE * 1.4f);
                        v.decay_rate = 0.999992f;
                        break;
                    case HALO_PAD: v.attack_rate = 1.0f / (SAMPLE_RATE * 1.8f);
                        v.decay_rate = 0.999995f;
                        break;
                    case AMBIENT_CLOUD: v.attack_rate = 1.0f / (SAMPLE_RATE * 2.5f);
                        v.decay_rate = 0.999995f;
                        v.fm_ratio = 1.5f;
                        v.fm_depth = 0.05f;
                        break;
                    case DEEP_DRONE: v.attack_rate = 1.0f / (SAMPLE_RATE * 2.0f);
                        v.decay_rate = 0.999996f;
                        break;
                    case CHIME_PAD: v.attack_rate = 1.0f / (SAMPLE_RATE * 0.5f);
                        v.decay_rate = 0.999980f;
                        v.fm_ratio = 3.5f;
                        v.fm_depth = 0.4f;
                        break;
                    case GHOST_CHOIR: v.attack_rate = 1.0f / (SAMPLE_RATE * 1.5f);
                        v.decay_rate = 0.999990f;
                        break;
                    case LUNAR_DUST: v.attack_rate = 1.0f / (SAMPLE_RATE * 0.6f);
                        v.decay_rate = 0.999985f;
                        break;
                    default: break;
                }
                notes_triggered++;
            }
        }

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
                if (v.env < 0.0001f) {
                    v.env = 0;
                    v.env_state = 0;
                    continue;
                }
            }

            v.vibPhase += TWO_PI * 3.0f / SAMPLE_RATE;
            float vib = 1.0f + 0.001f * std::sin(v.vibPhase);
            float dphi = TWO_PI * (v.freq * vib) / SAMPLE_RATE;
            float sample = 0;

            switch (v.type) {
                case SINE_PAD:
                    sample = std::sin(v.phase) * 0.6f + std::sin(v.phase2) * 0.2f + std::sin(v.phase3) * 0.2f;
                    v.phase += dphi;
                    v.phase2 += dphi * v.detune;
                    v.phase3 += dphi * 0.5f;
                    break;
                case SOFT_FM:
                    sample = std::sin(v.phase + v.fm_depth * v.env * std::sin(v.phaseMod)) * 0.7f + std::sin(v.phase) *
                             0.3f;
                    v.phase += dphi;
                    v.phaseMod += dphi * v.fm_ratio;
                    break;
                case WARM_PAD:
                    sample = std::sin(v.phase2) * 0.6f + (std::asin(std::sin(v.phase)) * 0.636f) * 0.4f;
                    v.phase += dphi;
                    v.phase2 += dphi * v.detune;
                    break;
                case CELLO_PAD:
                    sample = std::sin(v.phase) * 0.5f + std::sin(v.phase2) * 0.25f + std::sin(v.phase3) * 0.25f;
                    v.phase += dphi;
                    v.phase2 += dphi * (1.0f + v.detune * 0.001f);
                    v.phase3 += dphi * (1.0f - v.detune * 0.001f);
                    break;
                case DEEP_DRONE:
                    sample = std::sin(v.phase) * 0.6f + std::sin(v.phase2) * 0.3f + std::sin(v.phase3) * 0.1f;
                    v.phase += dphi;
                    v.phase2 += dphi * 0.5f;
                    v.phase3 += dphi * 1.002f;
                    break;
                case CHIME_PAD:
                    sample = std::sin(v.phase + std::sin(v.phaseMod) * v.fm_depth) * 0.6f + std::sin(v.phase2) * 0.4f;
                    v.phase += dphi;
                    v.phaseMod += dphi * v.fm_ratio;
                    v.phase2 += dphi * v.detune;
                    break;
                case GHOST_CHOIR:
                    sample = std::sin(v.phase) * 0.4f + std::sin(v.phase2) * 0.3f + std::sin(v.phase3) * 0.3f;
                    v.phase += dphi;
                    v.phase2 += dphi * 1.005f;
                    v.phase3 += dphi * 0.995f;
                    break;
                case LUNAR_DUST:
                    sample = std::sin(v.phase) * 0.3f + std::sin(v.phase2) * 0.3f + std::sin(v.phase3) * 0.2f +
                             std::sin(v.phase4) * 0.2f;
                    v.phase += dphi;
                    v.phase2 += dphi * 2.0f;
                    v.phase3 += dphi * 3.0f;
                    v.phase4 += dphi * 4.0f;
                    break;
                default:
                    sample = std::sin(v.phase);
                    v.phase += dphi;
                    break;
            }

            if (v.phase > TWO_PI) v.phase -= TWO_PI;
            if (v.phase2 > TWO_PI) v.phase2 -= TWO_PI;
            if (v.phase3 > TWO_PI) v.phase3 -= TWO_PI;
            if (v.phase4 > TWO_PI) v.phase4 -= TWO_PI;
            if (v.phaseMod > TWO_PI) v.phaseMod -= TWO_PI;

            v.angle += v.angleSpeed;
            if (v.angle > TWO_PI) v.angle -= TWO_PI;
            else if (v.angle < 0) v.angle += TWO_PI;

            float thetaL = v.angle + static_cast<float>(M_PI / 2.0);
            float thetaR = v.angle - static_cast<float>(M_PI / 2.0);

            float distL = 1.0f - std::cos(thetaL);
            float distR = 1.0f - std::cos(thetaR);

            float delayL = distL * 11.0f;
            float delayR = distR * 11.0f;

            v.delay3d.write(sample);
            float sL = v.delay3d.read(delayL);
            float sR = v.delay3d.read(delayR);

            float ampL = 1.0f - (distL * 0.35f);
            float ampR = 1.0f - (distR * 0.35f);

            float cutL = 0.25f + 0.75f * (ampL / 1.0f);
            float cutR = 0.25f + 0.75f * (ampR / 1.0f);

            v.lpL += (sL - v.lpL) * cutL;
            v.lpR += (sR - v.lpR) * cutR;

            float finalVolume = v.amp * v.env * 0.12f;
            mix_L += v.lpL * ampL * finalVolume;
            mix_R += v.lpR * ampR * finalVolume;
        }

        float revL, revR;
        reverb.process(mix_L, mix_R, revL, revR);

        float wet = 0.85f;
        float outL = mix_L * (1.0f - wet * 0.2f) + revL * wet;
        float outR = mix_R * (1.0f - wet * 0.2f) + revR * wet;

        filter.process(outL, outR, 0.40f);

        outL = std::tanh(outL * 1.35f) / 1.35f;
        outR = std::tanh(outR * 1.35f) / 1.35f;

        outL = dcL.process(outL);
        outR = dcR.process(outR);

        fOut[i * 2 + 0] = outL;
        fOut[i * 2 + 1] = outR;
    }
}

void process_raw_bytes(const uint8_t *buffer, int size) {
    int ip_offset = 0;
#ifdef __linux__
    ip_offset = 14;
#endif

    if (size < ip_offset + static_cast<int>(sizeof(IPv4Header))) return;
    const auto *ip = reinterpret_cast<const IPv4Header *>(buffer + ip_offset);
    if ((ip->version_ihl >> 4) != 4) return;
    if (ip->protocol != 6 && ip->protocol != 17) return;

    static auto last_note = std::chrono::steady_clock::now();
    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_note).count() < 60) return;
    last_note = now;

    const auto *dst = reinterpret_cast<const uint8_t *>(&ip->dst_ip);
    const uint16_t pkt_len = ntohs(ip->total_length);
    const uint16_t id = ntohs(ip->id);
    const uint16_t csum = ntohs(ip->checksum);

    int scale_idx;
    float freq;

    if (pkt_len > 1000) {
        scale_idx = (id ^ csum) % 8;
        freq = SCALE[scale_idx];
    } else if (pkt_len < 100) {
        scale_idx = 8 + ((id ^ csum) % 8);
        freq = SCALE[scale_idx];
    } else {
        scale_idx = (id ^ csum ^ pkt_len) % SCALE_SIZE;
        freq = SCALE[scale_idx];
    }

    const auto vtype = static_cast<VoiceType>((id + csum + pkt_len) % NUM_TYPES);

    const float norm = std::min(static_cast<float>(pkt_len) / 1500.0f, 1.0f);
    const float vol = 0.20f + std::log1p(norm * 4.0f) / std::log1p(4.0f) * 0.15f;

    const float startAngle = (id + dst[2]) % 360 * (TWO_PI / 360.0f);

    const int head = q_head.load();
    if (const int next = (head + 1) % QUEUE_SIZE; next != q_tail.load()) {
        note_queue[head] = {freq, vol, vtype, startAngle};
        q_head.store(next);
    }

    const char *names[] = {
        "Pad  ", "FMPad", "Warm ", "Wind ", "Cryst", "Cello",
        "Halo ", "Cloud", "Drone", "Chime", "Choir", "Dust "
    };

    const char *colors[] = {
        "\033[34m", "\033[33m", "\033[32m", "\033[36m",
        "\033[97m", "\033[35m", "\033[93m", "\033[95m",
        "\033[90m", "\033[96m", "\033[94m", "\033[92m"
    };

    const int bars = static_cast<int>(vol * 80);
    std::cout << colors[static_cast<int>(vtype)]
            << "[+] " << std::left << std::setw(6) << names[static_cast<int>(vtype)]
            << "| " << std::right << std::setw(6) << static_cast<int>(freq) << "Hz  "
            << std::string(bars, '=')
            << "\033[0m\n";
}

int main() {
#ifdef _WIN32
    HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD dwMode = 0;
    GetConsoleMode(hOut, &dwMode);
    SetConsoleMode(hOut, dwMode | ENABLE_VIRTUAL_TERMINAL_PROCESSING);
#endif

    std::cout << "\033[96m  === NetSymphony ===\n\033[0m\n";
    std::cout << "  Instruments: Pad | FMPad | Warm | Wind | Crystal | Cello\n";
    std::cout << "               Halo | Cloud | Drone | Chime | Choir | Dust\n\n";

#ifdef _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cerr << "WSAStartup failed.\n";
        return 1;
    }
#endif

    SOCKET sniffer = INVALID_SOCKET;
    bool socket_ready = false;

#ifdef _WIN32
    sniffer = socket(AF_INET, SOCK_RAW, IPPROTO_IP);
    if (sniffer == INVALID_SOCKET) {
        std::cerr << "\033[91m[ERROR] Failed to create raw socket.\n";
        std::cerr << "        You must run this program as Administrator.\n\033[0m\n";
        std::cerr << "Solution:\n";
        std::cerr << "  1. Right-click the .exe file\n";
        std::cerr << "  2. Select 'Run as administrator'\n\n";
        WSACleanup();
        return 1;
    }

    char hostname[256];
    if (gethostname(hostname, sizeof(hostname)) == SOCKET_ERROR) {
        std::cerr << "Cannot get hostname.\n";
        closesocket(sniffer);
        WSACleanup();
        return 1;
    }

    hostent *local = gethostbyname(hostname);
    if (!local) {
        std::cerr << "Cannot resolve hostname.\n";
        closesocket(sniffer);
        WSACleanup();
        return 1;
    }

    std::cout << "=== Available Network Interfaces ===\n";
    std::vector<in_addr> interfaces;
    int num_ifaces = 0;

    while (local->h_addr_list[num_ifaces] && num_ifaces < 10) {
        in_addr addr{};
        addr.s_addr = *reinterpret_cast<u_long *>(local->h_addr_list[num_ifaces]);
        interfaces.push_back(addr);
        std::cout << "[" << num_ifaces << "] " << inet_ntoa(addr) << "\n";
        num_ifaces++;
    }

    if (num_ifaces == 0) {
        std::cerr << "No network interfaces found.\n";
        closesocket(sniffer);
        WSACleanup();
        return 1;
    }

    int choice = 0;
    if (num_ifaces > 1) {
        std::cout << "\nSelect interface (0-" << (num_ifaces - 1) << "): ";
        std::cin >> choice;
        if (choice < 0 || choice >= num_ifaces) choice = 0;
    }

    sockaddr_in dest{};
    dest.sin_family = AF_INET;
    dest.sin_addr = interfaces[choice];
    dest.sin_port = 0;

    if (bind(sniffer, reinterpret_cast<sockaddr *>(&dest), sizeof(dest)) == SOCKET_ERROR) {
        std::cerr << "Failed to bind socket to interface.\n";
        closesocket(sniffer);
        WSACleanup();
        return 1;
    }

    DWORD dwBytesRet = 0;
    DWORD dwBufferLen[10];
    DWORD dwBufferLen_Len = sizeof(dwBufferLen);
    int j = SIO_RCVALL_ON;

    if (WSAIoctl(sniffer, SIO_RCVALL, &j, sizeof(j), dwBufferLen, dwBufferLen_Len, &dwBytesRet, nullptr, nullptr) ==
        SOCKET_ERROR) {
        std::cerr << "\033[91m[ERROR] WSAIoctl SIO_RCVALL failed.\n";
        std::cerr << "        Administrator rights are required.\n\033[0m\n";
        closesocket(sniffer);
        WSACleanup();
        return 1;
    }

    socket_ready = true;
    std::cout << "\n\033[92m✓ Socket initialized successfully\033[0m\n";
    std::cout << "Listening on: " << inet_ntoa(interfaces[choice]) << "\n\n";

#elif defined(__linux__)
    sniffer = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
    if (sniffer == INVALID_SOCKET) {
        std::cerr << "\033[91m[ERROR] Failed to create raw socket.\n";
        std::cerr << "        You must run this program with sudo or as root.\n";
        std::cerr << "\033[0m\n";
        std::cerr << "Solution: Run with 'sudo ./netsymphony'\n\n";
        return 1;
    }

    std::cout << "=== Available Network Interfaces ===\n";
    struct ifaddrs *ifaddr = nullptr;
    if (getifaddrs(&ifaddr) == -1) {
        perror("getifaddrs");
        closesocket(sniffer);
        return 1;
    }

    std::vector<std::pair<std::string, std::string> > interfaces;
    for (struct ifaddrs *ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr) continue;
        if (ifa->ifa_addr->sa_family == AF_INET) {
            char addr[INET_ADDRSTRLEN] = {0};
            struct sockaddr_in *sin = reinterpret_cast<struct sockaddr_in *>(ifa->ifa_addr);
            inet_ntop(AF_INET, &sin->sin_addr, addr, sizeof(addr));
            bool exists = false;
            for (auto &p: interfaces)
                if (p.first == ifa->ifa_name) {
                    exists = true;
                    break;
                }
            if (!exists) {
                interfaces.emplace_back(ifa->ifa_name, addr);
                std::cout << "[" << (interfaces.size() - 1) << "] " << ifa->ifa_name << " " << addr << "\n";
            }
        }
    }

    freeifaddrs(ifaddr);

    if (interfaces.empty()) {
        std::cerr << "No network interfaces found.\n";
        closesocket(sniffer);
        return 1;
    }

    int choice = 0;
    if (interfaces.size() > 1) {
        std::cout << "\nSelect interface (0-" << (interfaces.size() - 1) << "): ";
        std::cin >> choice;
        if (choice < 0 || choice >= static_cast<int>(interfaces.size())) choice = 0;
    }

    struct sockaddr_ll sll;
    memset(&sll, 0, sizeof(sll));
    sll.sll_family = AF_PACKET;
    sll.sll_protocol = htons(ETH_P_ALL);
    unsigned int ifindex = if_nametoindex(interfaces[choice].first.c_str());
    if (ifindex == 0) {
        std::cerr << "Failed to get interface index for " << interfaces[choice].first << "\n";
        closesocket(sniffer);
        return 1;
    }
    sll.sll_ifindex = static_cast<int>(ifindex);

    if (bind(sniffer, reinterpret_cast<struct sockaddr *>(&sll), sizeof(sll)) == SOCKET_ERROR) {
        perror("bind");
        closesocket(sniffer);
        return 1;
    }

    socket_ready = true;
    std::cout << "\n\033[92m✓ Socket initialized successfully (AF_PACKET)\033[0m\n\n";

#else
    sniffer = socket(AF_INET, SOCK_RAW, IPPROTO_TCP);
    if (sniffer == INVALID_SOCKET) {
        std::cerr << "\033[91m[ERROR] Failed to create raw socket.\n";
        std::cerr << "        You may need elevated privileges.\n\033[0m\n";
        return 1;
    }
    socket_ready = true;
    std::cout << "\n\033[92m✓ Socket initialized successfully\033[0m\n\n";
#endif

    if (!socket_ready) {
        std::cerr << "Failed to initialize socket.\n";
        if (sniffer != INVALID_SOCKET) closesocket(sniffer);
#ifdef _WIN32
        WSACleanup();
#endif
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
        std::cerr << "\033[91m[ERROR] Audio device initialization failed.\n\033[0m";
        closesocket(sniffer);
#ifdef _WIN32
        WSACleanup();
#endif
        return 1;
    }

    if (ma_device_start(&device) != MA_SUCCESS) {
        std::cerr << "\033[91m[ERROR] Failed to start audio device.\n\033[0m";
        ma_device_uninit(&device);
        closesocket(sniffer);
#ifdef _WIN32
        WSACleanup();
#endif
        return 1;
    }

    std::cout << "\033[96m═══════════════════════════════════════════\033[0m\n";
    std::cout << "\033[96m  Listening ...                            \033[0m\n";
    std::cout << "\033[96m═══════════════════════════════════════════\033[0m\n\n";
    std::cout << "Press Ctrl+C to stop...\n\n";

    uint8_t buffer[65536];
    int received = 0;

    while (true) {
        try {
#ifdef _WIN32
            received = recvfrom(sniffer, reinterpret_cast<char *>(buffer), sizeof(buffer), 0, nullptr, nullptr);
#else
            received = recvfrom(sniffer, reinterpret_cast<char *>(buffer), sizeof(buffer), 0, nullptr, nullptr);
#endif

            if (received > 0) {
                process_raw_bytes(buffer, received);
            } else if (received == SOCKET_ERROR) {
#ifdef _WIN32
                int err = WSAGetLastError();
                if (err == WSAEINTR) {
                    break;
                }
                std::cerr << "Socket error: " << err << "\n";
#else
                std::cerr << "Socket error\n";
#endif
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        } catch (const std::exception &e) {
            std::cerr << "Exception in packet processing: " << e.what() << "\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    std::cout << "\n\033[93mShutting down...\033[0m\n";
    ma_device_uninit(&device);
    closesocket(sniffer);

#ifdef _WIN32
    DWORD dwBytesRet2 = 0;
    int k = SIO_RCVALL_OFF;
    WSAIoctl(sniffer, SIO_RCVALL, &k, sizeof(k), nullptr, 0, &dwBytesRet2, nullptr, nullptr);
    WSACleanup();
#endif

    std::cout << "\033[92mCleanup complete. Goodbye!\033[0m\n";
    return 0;
}
