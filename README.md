## Project Overview
This repository contains the implementation for **Application C (3D Control for a drone using a single virtual camera)**. 

### Perception Architecture
While the project primary relies on **Depth Anything V2** for 3D spatial awareness, we discovered a core limitation in monocular depth networks regarding thin structures and homogeneous surfaces (such as plain walls/pillars) due to over-smoothing. 

To overcome this and ensure 100% flight stability, we implemented a hybrid perception suite:
1. **Depth Anything V2 (AI):** Provides global 3D distance metrics.
2. **Canny Edge Detection (Classical CV):** Provides real-time, low-latency boundary lines to detect narrow or featureless obstacles instantly.
