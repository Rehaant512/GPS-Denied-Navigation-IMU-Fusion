
# GPS-Denied Navigation — IMU + Magnetometer Sensor Fusion (C++)

Dead reckoning system for autonomous drone navigation in GPS-denied environments.

The system fuses data from an IMU and a high-precision magnetometer using Kalman filtering and quaternion mathematics to produce stable, tilt-compensated heading estimates without any reliance on GPS.

---

## The Problem

Standard drone navigation depends on GPS. In indoor environments, warehouses, or RF-congested areas, GPS is unavailable or unreliable. This project implements dead reckoning — estimating position and heading purely from onboard sensors — to maintain stable orientation and directional control in these conditions.

---

## System Architecture

```
IMU (Quaternion) ──► Kalman Filter ──► Quaternion Rotation ──► Earth-Frame Heading
                                              ▲
Magnetometer (Raw) ──► Kalman Filter ──────────
```

**Sensor fusion pipeline:**
1. IMU provides orientation as a quaternion (rotation in 3D space)
2. Magnetometer provides raw magnetic field vector
3. Kalman filters applied independently to both data streams to reject noise
4. Magnetometer vector rotated from body-frame to earth-frame using the quaternion
5. Earth-frame yaw computed via `atan2` — tilt-compensated and drift-resistant

---

## Implementations

Two hardware variants are provided, implementing the same algorithm:

### `dead_reckoning_BNO08x.ino` — High-Accuracy IMU
- **IMU:** Bosch BNO08x (ARVR-stabilized rotation vector)
- **Magnetometer:** PNI RM3100 (high-precision, low-noise)
- **Best for:** Flight-critical applications requiring maximum accuracy

### `dead_reckoning_MPU6050.ino` — Common IMU
- **IMU:** InvenSense MPU6050 (DMP-based quaternion output via FIFO)
- **Magnetometer:** PNI RM3100
- **Best for:** Cost-sensitive or hobbyist-grade setups

Both implementations share the same sensor fusion logic — swapping hardware is a drop-in change to the IMU initialization and quaternion source.

---

## Key Technical Concepts

**Kalman Filtering**
Applied independently to all four quaternion components (`r, i, j, k`) and all three magnetometer axes (`x, y, z`) to smooth sensor noise before fusion.

**Quaternion-Based Tilt Compensation**
A naive magnetometer yaw reading is corrupted when the drone is not level. This is solved by rotating the magnetometer's body-frame vector into the earth frame using the full 3D orientation quaternion from the IMU — giving accurate yaw regardless of pitch/roll.

```cpp
void rotateVectorByQuaternion(float x, float y, float z,
                               float qr, float qi, float qj, float qk,
                               float &xr, float &yr, float &zr);
```

**Yaw Computation**
```cpp
float yaw = atan2(my_earth, mx_earth) * RAD_TO_DEG;
if (yaw < 0) yaw += 360; // Normalize to [0°, 360°]
```

---

## Hardware

| Component | Part | Interface |
|-----------|------|-----------|
| IMU (primary) | Bosch BNO08x | I2C |
| IMU (alternate) | InvenSense MPU6050 | I2C |
| Magnetometer | PNI RM3100 | I2C |
| MCU | Arduino-compatible | — |

---

## Dependencies

Install via Arduino Library Manager:

- `Adafruit BNO08x` (for BNO08x variant)
- `MPU6050` by Electronic Cats (for MPU6050 variant)
- `SimpleKalmanFilter` by Denys Sene
- `QRM3100` (RM3100 magnetometer driver)
- `Wire` (built-in I2C library)

---

## How to Build and Flash

1. Open the relevant `.ino` file in Arduino IDE
2. Install dependencies via Library Manager
3. Select your target board (e.g., Arduino Mega for Serial1 support)
4. Compile and flash

Serial output at 115200 baud will display real-time pitch, roll, and tilt-compensated yaw.

---

## Output Format

```
Pitch: 2.34°, Roll: -1.12°, Yaw: 143.67°
Pitch: 2.31°, Roll: -1.09°, Yaw: 143.70°
```
