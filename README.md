<div align="center">

```
  ～  NetSymphony  ～
```

**Live network traffic → generative ambient music**

[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://isocpp.org/)
[![Platform](https://img.shields.io/badge/platform-Linux%20%7C%20Windows%20%7C%20macOS-lightgrey.svg)]()

</div>

---

NetSymphony captures every IP packet flowing through your network interface and translates it into a voice in a
continuously evolving ambient soundscape. Packet size becomes pitch, protocol becomes timbre, TTL and IP addresses
become panning and volume. The result is a real-time generative piece that is unique to your machine and moment in time.

## How it works

```
NIC (raw socket)
      │
      ▼
IPv4 header parsing
  ├── packet size   →  pitch   (18-note undertone scale)
  ├── protocol      →  voice type
  ├── TTL           →  voice type refinement
  ├── src/dst IP    →  pan position + pitch offset
  └── DSCP / TOS    →  voice type override
      │
      ▼
Ring buffer (lock-free, 512 events)
      │
      ▼
Audio thread  (miniaudio callback, 44.1 kHz stereo)
  ├── Up to 24 simultaneous voices
  ├── Freeverb-style reverb (6 combs + 3 allpass per channel)
  ├── Slow global LFO "breathing" effect
  ├── tanh soft-saturation
  └── DC blocking filters
```

## Voice types

| Voice            | Trigger condition             | Character                   |
| ---------------- | ----------------------------- | --------------------------- |
| **Sine Pad**     | Default / fallback            | Warm, layered sines         |
| **Soft FM**      | TTL > 100 (Windows / routers) | Metallic, brassy            |
| **Smooth Pluck** | Small TCP packets             | Clear, brief plucked string |
| **Flute Pad**    | Large UDP (streaming)         | Breathy, woody              |
| **Glass Pad**    | Small–medium UDP              | Bell-like, crystalline      |
| **String Pad**   | Large TCP, TTL ≤ 64           | Lush string ensemble swell  |
| **Choir Pad**    | Even src IP, medium size      | Vocal, choral texture       |
| **Shimmer**      | Tiny UDP or DSCP-marked       | High, airy shimmer          |

## Requirements

- **C++17** compiler (GCC 8+, Clang 7+, MSVC 2019+)
- **Raw socket privileges** - `sudo` on Linux/macOS, **Run as Administrator** on Windows

## Building

### Linux

```bash
git clone https://github.com/dest4590/NetSymphony
cd NetSymphony

g++ -O2 -std=c++17 -o netsymphony main.cpp -lpthread -ldl -lm
```

### Windows (MSVC)

```bat
git clone https://github.com/dest4590/NetSymphony
cd NetSymphony

cl /O2 /std:c++17 main.cpp ws2_32.lib
```

### macOS

```bash
g++ -O2 -std=c++17 -o netsymphony main.cpp -lpthread -ldl -lm
```

> macOS note: full promiscuous capture requires `/dev/bpf` access. The build works but may capture fewer packet types
> than Linux.

## Running

```bash
# Linux / macOS
sudo ./netsymphony

# Windows - open a terminal as Administrator, then:
netsymphony.exe
```

On Windows you will be shown a list of detected network interfaces and asked to choose one.

## Console output

Each packet that triggers a note prints a coloured bar:

```
Flute  144Hz  ·····················
Pluck   96Hz  ·············
Shmr   192Hz  ··········································
Strng   72Hz  ·······
```

The bar width is proportional to volume (which comes from packet size). Colour corresponds to voice type.

## Architecture notes

### Synthesis

All synthesis is additive or subtractive with no external sample data. The Karplus–Strong pluck is the only algorithm
that requires a heap allocation (the delay line), and that allocation happens at note-on time, not inside the per-sample
loop.

### Reverb

The reverb is a classic Schroeder/Moorer "Freeverb" topology: six parallel comb filters (with one-pole damping inside
the feedback loop) followed by three series Schroeder allpass diffusers. Left and right channels use prime delay lengths
offset by 31 samples to produce stereo width from a mono source.

## Project structure

```
NetSymphony/
├── main.cpp        # Everything - packet capture, synthesis, reverb
├── miniaudio.h
└── README.md
```
