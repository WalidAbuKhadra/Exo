# Exo

**Real-time, AprilTag-calibrated monocular motion capture system powered by the NVIDIA AR SDK.** Converts webcam feed into world-aligned 3D skeletal data and broadcasts via OSC.

**Context:** Built as the input backend for a live generative art installation, driving visuals in TouchDesigner/Unreal Engine.

## Core Architecture
*   **Language:** C++20 (MSVC).
*   **Pipeline:** Threaded Producer-Consumer model (`Capture` → `Inference` → `Output` || `GUI`).
*   **Concurrency:** Custom **Lock-Free 4-Slot Hybrid Buffer** (Write/Shared/Read/Peek).
    *   **Mechanism:** Writer continuously updates the `Shared` slot. Consumer atomically exchanges `Shared` ↔ `Read` to fetch latest data. Upon GUI request, the stale `Read` buffer swaps with `Peek`; the displaced `Peek` buffer recycles to the writer pool.
    *   **Context Isolation:** Capture, Inference, Output, and GUI operate asynchronously at independent clock rates.
    *   **Hardware Optimization:** Atomic indices are padded to cache lines (`std::hardware_destructive_interference_size`) to eliminate false sharing.

## Features
*   **Automated Calibration:** OpenCV AprilTag integration instantly aligns camera space to physical world coordinates.
*   **Inference:** Hardware-accelerated NVIDIA Body Pose 3D tracking.
*   **Networking:** Low-latency UDP/OSC broadcasting.
*   **Modern Build:** CMake Presets, Ninja, Vcpkg manifest mode.

## Build Instructions
**Prerequisites:** Windows 10/11, NVIDIA RTX GPU, Visual Studio 2022.

1.  **Clone:**
    ```bash
    git clone https://github.com/USERNAME/Exo.git
    ```
2.  **Link SDK:**
    *   *Note:* Proprietary binaries are excluded from source control.
    *   Download NVIDIA AR SDK.
    *   Place files in `third_party/ARSDK/`.
3.  **Compile:**
    ```bash
    cmake --preset x64-release
    cmake --build --preset x64-release
    ```

## License
MIT