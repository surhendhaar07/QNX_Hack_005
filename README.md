# QNX Intelligent Adaptive Cruise Control (ACC)

A **QNX-based real-time Adaptive Cruise Control prototype** designed to
demonstrate deterministic task scheduling, inter-process communication,
sensor processing, safety supervision, timing analysis, and actuator
control on a Raspberry Pi target.

> **Project status:** The real-time ACC software architecture and
> HC-SR04 sensing pipeline are implemented. The current actuator layer
> identifies the L298N GPIO interface (ENA=GPIO18, IN1=GPIO17,
> IN2=GPIO27), but the QNX Raspberry Pi motor/PWM hardware backend is
> intentionally disabled until the target GPIO/PWM interface is
> verified. Therefore, the current software may command a motor while
> the actuator hardware layer safely keeps the physical output disabled.

------------------------------------------------------------------------

## 1. Project Overview

The system models an automotive-style ACC pipeline:

``` text
             +------------------+
             |    HC-SR04       |
             | Ultrasonic Sensor|
             +--------+---------+
                      |
                      v
             +------------------+
             |      RADAR       |
             | Distance Acquire |
             +--------+---------+
                      |
                POSIX IPC Queue
                      |
                      v
             +------------------+
             |    TRACKING      |
             | Relative Speed   |
             +--------+---------+
                      |
                POSIX IPC Queue
                      |
                      v
             +------------------+
             |   CONTROLLER     |
             | Speed / Distance |
             | Control Decision |
             +--------+---------+
                      |
                POSIX IPC Queue
                      |
                      v
             +------------------+
             |     SAFETY       |
             | Fault / E-STOP   |
             | Command Approval |
             +--------+---------+
                      |
                POSIX IPC Queue
                      |
                      v
             +------------------+
             |    ACTUATOR      |
             | L298N + PWM/GPIO |
             +--------+---------+
                      |
                      v
                  DC MOTOR

             +------------------+
             |   SUPERVISOR     |
             | Process / System |
             | Health Monitoring |
             +------------------+
```

The design follows a **safety-first control path**: the Controller does
not directly control the motor. Actuator commands are accepted only
after passing through the Safety layer.

------------------------------------------------------------------------

## 2. Objectives

-   Implement an ACC control pipeline on **QNX Neutrino RTOS**.
-   Demonstrate **POSIX real-time threads** and deterministic periodic
    execution.
-   Use **POSIX message queues** for process-to-process communication.
-   Measure:
    -   task execution time
    -   response time
    -   IPC latency
    -   deadline violations
-   Detect sensor faults and stale data.
-   Provide an explicit **safe-stop / emergency-stop path**.
-   Interface an **HC-SR04 ultrasonic sensor** with Raspberry Pi GPIO.
-   Interface an **L298N motor driver** for the physical actuator stage.
-   Provide a modular codebase suitable for further real-time and
    automotive experimentation.

------------------------------------------------------------------------

## 3. Hardware

  Component               Purpose
  ----------------------- --------------------------------------
  Raspberry Pi 4/5        QNX target platform
  HC-SR04                 Forward-distance measurement
  L298N                   DC motor driver
  DC Motor                Vehicle/motorized prototype actuator
  Breadboard              Prototyping
  Jumper wires            Interconnections
  External motor supply   Motor power

### GPIO Assignment

  Device    Signal     Raspberry Pi GPIO   Physical Pin
  --------- -------- ------------------- --------------
  HC-SR04   TRIG                  GPIO23         Pin 16
  HC-SR04   ECHO                  GPIO24         Pin 18
  L298N     ENA                   GPIO18         Pin 12
  L298N     IN1                   GPIO17         Pin 11
  L298N     IN2                   GPIO27         Pin 13

### Important Hardware Safety

**HC-SR04 ECHO is a 5 V signal. Raspberry Pi GPIO is 3.3 V logic. Use a
suitable voltage divider or level shifter between ECHO and GPIO24.**

Do not power the DC motor directly from the Raspberry Pi. Use an
appropriate external motor supply and ensure the Raspberry Pi and
motor-driver grounds are correctly referenced.

------------------------------------------------------------------------

## 4. Software Architecture

The project is divided into independent QNX processes:

  Process        Priority   Period Main Responsibility
  ------------ ---------- -------- ---------------------------------
  Supervisor           45   100 ms System/process supervision
  Safety               40    10 ms Safety validation and safe stop
  Actuator             35    60 ms Motor command application
  Controller           30    60 ms ACC control decision
  Tracking             25    60 ms Relative-speed calculation
  Radar                20    60 ms HC-SR04 acquisition

The implementation uses `SCHED_FIFO` real-time scheduling where
configured by the project.

### Control Flow

``` text
HC-SR04
   |
   v
Radar
   |
   | /acc_radar_q
   v
Tracking
   |
   | /acc_tracking_q
   v
Controller
   |
   | /acc_controller_q
   v
Safety
   |
   | /acc_safety_q
   v
Actuator
   |
   v
L298N
   |
   v
DC Motor
```

The Supervisor provides a separate system-level monitoring function.

------------------------------------------------------------------------

## 5. ACC Control Model

The current configuration defines:

``` text
Target speed             = 20 km/h
Safe distance            = 50 cm
Warning distance         = 30 cm
Emergency distance       = 15 cm
Sensor stale timeout     = 150 ms
Controller deadline      = 20 ms
```

Distance zones:

``` text
Distance >= 50 cm
        |
        v
Normal / cruising control


30 cm <= Distance < 50 cm
        |
        v
Distance-aware control


15 cm <= Distance < 30 cm
        |
        v
Warning / reduced-speed operation


Distance < 15 cm
        |
        v
Emergency / safe stop
```

The exact control response is determined by the Controller and Safety
state machine implemented in the source code.

------------------------------------------------------------------------

## 6. Safety Design

Safety is implemented as an independent processing stage between control
generation and physical actuation.

The Actuator receives commands from **Safety**, rather than directly
from the Controller.

A command can be rejected or forced to a safe state for conditions such
as:

-   invalid sensor data
-   sensor timeout
-   stale sensor data
-   controller deadline violation
-   invalid actuator command
-   IPC failure
-   process fault
-   emergency distance condition
-   shutdown

The safe actuator condition is:

``` text
Motor direction = STOP
PWM duty        = 0%
Actuator enable = 0
```

Example runtime behavior:

``` text
CONTROLLER
state=ACTIVE
duty=30.3%
enable=1

        |
        v

SAFETY
state=FAULT
fault=1
emergency=1

        |
        v

ACTUATOR
direction=STOP
duty=0.0%
```

This separation makes safety enforcement independent of the normal
control request.

------------------------------------------------------------------------

## 7. Real-Time Design

The project uses QNX real-time concepts including:

-   POSIX threads
-   `SCHED_FIFO`
-   task priorities
-   periodic absolute-time scheduling
-   `CLOCK_MONOTONIC`
-   execution-time measurement
-   response-time measurement
-   deadline monitoring
-   CPU affinity
-   POSIX message queues
-   fault propagation and safe-stop behavior

Timing measurements are based on `CLOCK_MONOTONIC`, avoiding dependence
on wall-clock adjustments.

The timing subsystem supports:

``` text
Task start
   |
   v
Task execution
   |
   +---- execution time
   |
   +---- response time
   |
   +---- deadline check
```

IPC messages also carry timestamps so that communication latency can be
measured.

------------------------------------------------------------------------

## 8. Inter-Process Communication

The project uses **POSIX message queues**.

Configured queues include:

``` text
/acc_radar_q
/acc_tracking_q
/acc_controller_q
/acc_safety_q
/acc_actuator_q
/acc_supervisor_q
```

Message headers contain information such as:

-   message type
-   sender PID
-   sequence number
-   timestamp

The sender side uses non-blocking message-queue operation so that a full
queue does not indefinitely block a real-time task.

------------------------------------------------------------------------

## 9. Sensor Processing

The HC-SR04 measurement sequence is:

``` text
TRIG HIGH (~10 us)
       |
       v
Ultrasonic pulse transmitted
       |
       v
Wait for ECHO HIGH
       |
       v
Measure ECHO pulse width
       |
       v
Calculate distance
       |
       v
Validate measurement
       |
       v
Send Radar data through IPC
```

Distance is calculated from the measured echo time using the speed of
sound and the round-trip factor.

The software does **not intentionally generate random or synthetic
sensor distances** when the real sensor read fails. Instead, sensor
faults are propagated through the ACC pipeline.

------------------------------------------------------------------------

## 10. Fault Handling

The system distinguishes between normal sensor operation and fault
conditions.

Typical sensor states include:

``` text
SENSOR_OK
SENSOR_TIMEOUT
SENSOR_INVALID
SENSOR_STALE
```

For example:

``` text
HC-SR04 timeout
      |
      v
Radar reports SENSOR_TIMEOUT
      |
      v
Tracking propagates sensor fault
      |
      v
Controller enters fault-safe state
      |
      v
Safety asserts fault / emergency stop
      |
      v
Actuator receives STOP
```

This behavior is important for demonstrating fault injection and
safety-oriented real-time design.

------------------------------------------------------------------------

## 11. Project Structure

A typical project organization is:

``` text
QNX_ACC/
├── src/
│   ├── QNX_ACC.c
│   │
│   ├── launcher/
│   │   └── main.c
│   │
│   ├── radar/
│   │   ├── radar.c
│   │   ├── radar.h
│   │   ├── radar_main.c
│   │   ├── hc_sr04.c
│   │   └── hc_sr04.h
│   │
│   ├── tracking/
│   │   ├── tracking.c
│   │   ├── tracking.h
│   │   └── tracking_main.c
│   │
│   ├── controller/
│   │   ├── controller.c
│   │   ├── controller.h
│   │   └── controller_main.c
│   │
│   ├── safety/
│   │   ├── safety.c
│   │   ├── safety.h
│   │   └── safety_main.c
│   │
│   ├── actuator/
│   │   ├── actuator.c
│   │   ├── actuator.h
│   │   ├── actuator_main.c
│   │   ├── l298n.c
│   │   ├── l298n.h
│   │   ├── pwm.c
│   │   └── pwm.h
│   │
│   ├── supervisor/
│   │   ├── supervisor.c
│   │   ├── supervisor.h
│   │   └── supervisor_main.c
│   │
│   └── common/
│       ├── acc_config.h
│       ├── acc_types.h
│       ├── ipc.c
│       ├── ipc.h
│       ├── ipc_types.h
│       ├── timing.c
│       ├── timing.h
│       ├── timing_types.h
│       ├── qnx_affinity.c
│       ├── qnx_affinity.h
│       ├── qnx_priority.c
│       └── qnx_priority.h
│
├── README.md
└── ...
```

------------------------------------------------------------------------

## 12. Building

The project is intended to be built for an **AArch64 QNX target**.

The current build configuration uses:

``` text
qcc
-Vgcc_ntoaarch64le
```

A typical build from the QNX development environment is:

``` bash
make -j12 all
```

The project can be built from QNX Momentics or an equivalent QNX
command-line build environment.

After a successful build, copy the required executables to the Raspberry
Pi QNX target.

------------------------------------------------------------------------

## 13. Running

On the QNX target:

``` bash
./acc_launcher
```

The launcher starts the system processes in the required architecture.

A normal startup sequence resembles:

``` text
[LAUNCHER] Starting ACC processes...

[LAUNCHER] Started Supervisor
[LAUNCHER] Started Actuator
[LAUNCHER] Started Safety
[LAUNCHER] Started Controller
[LAUNCHER] Started Tracking
[LAUNCHER] Started Radar
```

Then the individual tasks begin reporting their sequence numbers, sensor
values, states, IPC timing, and execution timing.

------------------------------------------------------------------------

## 14. Example Runtime Output

Example normal sensor operation:

``` text
RADAR: seq=1 distance=51.42 cm status=OK exec=7.742 ms

TRACKING:
distance=51.42 cm
relative_speed=0.00 cm/s
IPC=0.013 ms
exec=0.002 ms

CONTROLLER:
distance=51.42 cm
state=1
duty=30.3%
enable=1

SAFETY:
state=4
fault=1
emergency=1

ACTUATOR:
direction=STOP
duty=0.0%
```

The important point is that the Controller's requested output and the
Safety-approved actuator output are separate. A safety fault can
override a normal control request.

------------------------------------------------------------------------

## 15. Timing and Performance Measurement

The project is designed to support measurement of:

### Execution Time

Time spent executing a task.

``` text
execution_time =
    task_end - task_start
```

### IPC Latency

Time between message generation and message reception.

``` text
IPC latency =
    receiver_timestamp - sender_timestamp
```

### Deadline Monitoring

Each real-time task can compare its measured execution/response time
against its configured deadline.

Example:

``` text
Controller deadline = 20 ms

Execution <= 20 ms
        |
        v
Deadline OK

Execution > 20 ms
        |
        v
Deadline MISSED
        |
        v
Safety fault path
```

------------------------------------------------------------------------

## 16. Fault Injection and Testing

The architecture supports testing of conditions such as:

-   HC-SR04 timeout
-   invalid distance
-   stale sensor data
-   controller deadline violation
-   IPC failure
-   invalid actuator command
-   emergency-distance condition
-   process shutdown/fault

A representative fault output is:

``` text
RADAR: SENSOR FAULT
       |
       v
TRACKING: SENSOR FAULT
       |
       v
CONTROLLER: state=FAULT
       |
       v
SAFETY: fault=1 emergency=1
       |
       v
ACTUATOR: duty=0.0%
```

These tests are useful for demonstrating the safety path and the
real-time response of the system.

------------------------------------------------------------------------

## 17. Current Hardware Backend Status

### HC-SR04

The project includes a QNX GPIO implementation path for:

``` text
TRIG -> GPIO23
ECHO -> GPIO24
```

Runtime testing has demonstrated valid distance readings on the target
in configurations where the QNX GPIO interface is available.

### L298N / PWM

The actuator layer currently identifies:

``` text
ENA -> GPIO18
IN1 -> GPIO17
IN2 -> GPIO27
```

However, the current `l298n.c` and `pwm.c` deliberately keep the
hardware backend disabled until the exact Raspberry Pi QNX GPIO/PWM
interface is verified.

Runtime messages currently include:

``` text
[PWM] Hardware PWM backend not configured
[PWM] Output disabled

[L298N] Hardware backend not configured
[L298N] ENA GPIO: 18
[L298N] IN1 GPIO : 17
[L298N] IN2 GPIO : 27
[L298N] Motor output disabled
```

This is intentional: the software does not pretend that an unverified
motor interface is working.

------------------------------------------------------------------------

## 18. Troubleshooting

### `SENSOR FAULT`

Check:

1.  HC-SR04 power.
2.  Common ground.
3.  TRIG wiring to GPIO23.
4.  ECHO wiring to GPIO24.
5.  ECHO voltage-level conversion.
6.  QNX GPIO resource manager/backend.
7.  Sensor timeout behavior.

Typical message:

``` text
HC_SR04: timeout/error waiting for ECHO HIGH
```

### Motor does not rotate

First check whether the software is actually approving actuator
operation.

Look for:

``` text
CONTROLLER ... duty=...
SAFETY ... fault=...
ACTUATOR ... duty=...
```

If Safety reports:

``` text
fault=1
emergency=1
```

the Actuator is expected to stop the motor.

Also note that the current hardware backend reports:

``` text
Hardware backend not configured
```

so the physical motor output remains disabled until the QNX GPIO/PWM
implementation is completed and verified.

### `Message too long`

If POSIX message queues report:

``` text
mq_receive: Message too long
```

check that the queue's configured `mq_msgsize` is at least as large as
the actual `ipc_message_t` being transmitted.

The project contains an IPC message-size validation function for this
purpose.

### Deadline missed

Example:

``` text
CONTROLLER DEADLINE MISSED
execution=...
deadline=20 ms
```

Investigate:

-   blocking operations inside the real-time task
-   sensor polling duration
-   excessive logging
-   queue congestion
-   scheduling priority
-   CPU affinity
-   unexpected contention

------------------------------------------------------------------------

## 19. Design Principles

The project follows these principles:

-   **Safety before actuation**
-   **No fabricated sensor values during hardware faults**
-   **Independent real-time processes**
-   **Explicit IPC boundaries**
-   **Measured timing instead of assumed timing**
-   **Deadline monitoring**
-   **Fail-safe actuator behavior**
-   **Modular hardware abstraction**
-   **QNX/POSIX real-time mechanisms**
-   **No permanent runtime data storage**

------------------------------------------------------------------------

## 20. Technologies

-   **QNX Neutrino RTOS**
-   **QNX Momentics**
-   **C**
-   **POSIX Threads**
-   **POSIX Message Queues**
-   **SCHED_FIFO**
-   **CLOCK_MONOTONIC**
-   **QNX AArch64 toolchain**
-   **Raspberry Pi 4/5**
-   **HC-SR04 Ultrasonic Sensor**
-   **L298N Motor Driver**
-   **DC Motor**
-   **GPIO / PWM hardware interface**

------------------------------------------------------------------------

## 21. Development Roadmap

``` text
[x] QNX project setup
[x] Multi-process ACC architecture
[x] Radar process
[x] HC-SR04 sensing pipeline
[x] Tracking process
[x] Controller process
[x] Safety process
[x] Actuator process structure
[x] POSIX IPC
[x] Real-time timing measurement
[x] Deadline monitoring
[x] Supervisor framework
[x] Fault propagation
[ ] Verify GPIO18 hardware output
[ ] Complete QNX L298N GPIO backend
[ ] Complete QNX PWM backend
[ ] Physical motor validation
[ ] Full performance characterization
[ ] Extended fault-injection campaign
```

------------------------------------------------------------------------

## 22. Academic / Demonstration Scope

This project is intended as a **real-time systems and embedded/RTOS
prototype**, not as a production automotive safety system.

It demonstrates how an ACC-style application can be decomposed into
independently scheduled QNX processes with explicit IPC, timing
constraints, fault handling, and a safety-controlled actuator path.

The project is suitable for demonstrating:

-   RTOS concepts
-   process/thread scheduling
-   real-time IPC
-   sensor interfacing
-   control logic
-   safety mechanisms
-   fault injection
-   deadline analysis
-   execution-time analysis
-   embedded hardware integration

------------------------------------------------------------------------

## 23. License

Add the project's chosen license here before publishing the repository,
for example:

``` text
MIT License
```

or another license appropriate for the project.

------------------------------------------------------------------------

## 24. Acknowledgements

Built as a QNX real-time systems project using QNX Neutrino concepts,
POSIX APIs, Raspberry Pi hardware, HC-SR04 sensing, and L298N motor
control.

------------------------------------------------------------------------

## 25. Disclaimer

This is an educational/prototyping implementation. It is **not suitable
for installation in a real vehicle or use as an automotive
safety-critical control system** without the extensive hardware,
software, verification, validation, redundancy, diagnostics, and safety
engineering required for such applications.
