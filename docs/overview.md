# FlightControllerNanoV2 Architecture Overview


## 0) Quaternion and Frame Conventions

### 0.1 What `q` represents

orientation q is used as rotation operator for rotating vectors like this: 

- body -> earth: `v_e = q * v_b * q_conj`
- earth -> body: `v_b = q_conj * v_e * q`

where `q_conj` is quaternion conjugate.

`Madgwick::q` is used as the vehicle attitude quaternion such that gravity in body frame is compared against a fixed DOWN direction during correction.

- `Madgwick` builds correction from accelerometer directly against gravity model terms expressed in body coordinates.
- Initialization uses `quatFromTwoUnitVectors(g_body, DOWN)` with `DOWN = {0,0,1}`.

### 0.2 NED coordinate convention

- +X: North/forward direction of nav frame
- +Y: East/right direction of nav frame
- +Z: Down

Body-frame sign/mapping is fixed by `IMU_MAP_*` macros in `include/drivers/mpu9250.h` and applied in `MPU9250::read`.


## 1) High-Level Control Stack

Main loop lives in `src/main.cpp` and follows this sequence:

1. Read IMU (`imu_device.read`) and update attitude estimate (`madgw.update`).
2. Read and normalize pilot command (`receiver.read` + `receiver.normalize`).
3. Convert attitude/rate error into motor adjustments (`atti_stabilizer.compute_rpy_adjust`).
4. Mix throttle + attitude adjustments and write to motors (`motor_device.write`).
5. Run barometer state machine (`barometer.kick`, `barometer.read`).
6. Use remaining loop budget for optical-flow UART parsing (`optical_flow.kick`, `optical_flow.read`).
7. Busy-wait until fixed loop period ends.

This is a fixed-rate control loop design with low-priority background parsing in remaining time.

## 2) Madgwick filter for sensor fusion

Attitude estimation is implemented in:

- `include/madgwick.h`
- `src/madgwick.cpp`

Algorithm details in this codebase:

- State is quaternion `q` (body orientation).
- Prediction uses gyro integration: `q_dot = 0.5 * q * omega`.
- Correction uses a gradient-descent step from accelerometer gravity alignment.
- Gain `beta` (`MW_BETA` in `include/config.h`) controls correction strength.
- `Madgwick::update()` runs at loop cadence (`PERIOD_US`) and computes actual `dt` from `micros()`.

Initialization:

- `Madgwick(beta, imu_data.accel)` initializes quaternion from measured gravity vector to DOWN (`{0,0,1}`).

Important assumption:

- Accelerometer used for correction should represent gravity direction during low linear acceleration; aggressive translational acceleration will temporarily disturb correction.

## 3) Driver vs Device Architecture

- `drivers/*`: hardware-facing transport/protocol/register logic.
- `devices/*`: higher-level behavior, scheduling/throttling, filtering, and domain transforms.
- `include/interfaces.h`: interface contracts connecting both layers.

## 4) Modules

### IMU

- Driver: `src/drivers/mpu9250.cpp` (`MPU9250`)
- Device: `src/devices/imu.cpp` (`Imu`)

Function:

- Driver configures MPU9250 registers, reads burst accel/gyro, applies axis mapping/sign convention, gyro bias compensation, accel affine calibration (`bias`, `scale`).
- Device enforces read period gate (`period_us`, default `PERIOD_US`).

Convention:

- Axis mapping/sign is configured by `IMU_MAP_*` macros in `include/drivers/mpu9250.h`.
- Output `ImuData` is in body frame convention after mapping/sign.

### Barometer

- Driver: `src/drivers/ms5611.cpp` (`MS5611`)
- Device: `src/devices/barometer.cpp` (`Barometer`)

Function:

- Driver runs non-blocking conversion state machine (`kick`) for pressure/temperature and compensation math.
- Device runs startup calibration for ground pressure and low-pass filters altitude.

Timing:

- Device `kick()` is rate-limited to `BARO_ALT_HZ` (30 Hz).

### Optical Flow / Range (MTF02)

- Driver: `src/drivers/mtf02.cpp` (`MTF02`)
- Device: `src/devices/optical_flow.cpp` (`OpticalFlow`)

Function:

- Driver parses byte stream frame-by-frame (state machine + checksum).
- Device currently forwards parse/read calls and exposes helpers.

Scheduling:

- Main loop parses this as low priority only while loop time budget remains.

### Receiver

- Driver: `src/drivers/receiver.cpp` (`PPMReceiver`)
- Device: `src/devices/receiver.cpp` (`Receiver`)

Function:

- Driver acquires raw channel pulses and clamps startup-invalid values.
- Device normalizes channels to control-space targets (yaw rate + pitch/roll angles + normalized throttle) and applies limits.

### Motor Output

- Driver: `src/drivers/motors.cpp` (`Motor`)
- Device: `src/devices/motor.cpp` (`MotorDevice`)

Function:

- Device performs quad mix (`throttle + yaw/pitch/roll` per motor).
- Driver clamps to [0,1], converts to 12-bit raw PWM values, writes at configured PWM frequency.

### Stabilization / PID

- Logic: `src/pid.cpp`, declarations in `include/pid.h`

Function:

- Two-layer stabilization intent:
1. Outer: angle error -> target angular rates (nonlinear mapping).
2. Inner: body-rate PID tracks target rates against gyro.
- Outputs normalized yaw/pitch/roll adjustments for motor mixing.

## 5) Timing Model and Constraints

Global timing constants (`include/config.h`):

- `LOOP_HZ = 250`
- `PERIOD_US = 4000`
- `DT = 1/250`

How fixed-rate timing is implemented:

- At loop start: store `last_active = micros()`.
- Run control work.
- Run low-priority parsing while `(micros() - last_active) < (PERIOD_US - margin)`.
- Busy wait until full period elapsed.

Per-module timing gates:

- IMU device read returns false if called faster than configured period.
- Madgwick update also self-gates by `PERIOD_US` and uses measured `dt`.
- Barometer `kick()` self-gates to 30 Hz device period.
- MS5611 driver conversion uses non-blocking state progression by timestamp.

Design intent:

- Keep attitude/control path deterministic at 250 Hz.
- Push serial parsing and noncritical work into leftover slack.

## 6) Startup and Safety Behavior

In `setup()` (`src/main.cpp`):

1. Bring up Serial and I2C.
2. Setup IMU, barometer, optical flow.
3. Take initial IMU sample and initialize Madgwick quaternion.
4. Hold until throttle channel is low before enabling normal loop.

In `loop()`:

- If throttle is near zero, motors are set to zero and PID integrators are reset (`atti_stabilizer.reset`).

## 7) Accel Calibration Notes (6 Orientation)

Current accelerometer correction in driver uses affine per-axis correction in body frame:

- `a_corrected = (a_measured - accel_bias_g) * accel_scale`

6-orientation calibration routine (if enabled in your current working version) is intended to:

- Capture + and - gravity extremes for X/Y/Z body axes.
- Solve per-axis bias and scale so those faces map to approximately `+1g` and `-1g`.

## 8) Files

- Main orchestration: `src/main.cpp`
- Timing constants: `include/config.h`
- Interfaces/contracts: `include/interfaces.h`
- Attitude estimator: `src/madgwick.cpp`
- Stabilizer + PID: `src/pid.cpp`
- IMU mapping/calibration: `src/drivers/mpu9250.cpp`, `include/drivers/mpu9250.h`
- Baro pipeline: `src/devices/barometer.cpp`, `src/drivers/ms5611.cpp`
- Optical flow parser: `src/drivers/mtf02.cpp`
- Receiver normalization: `src/devices/receiver.cpp`
- Motor mixing/output: `src/devices/motor.cpp`, `src/drivers/motors.cpp`

