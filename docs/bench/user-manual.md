# IM6014 Motor Bench — user manual

This local desktop application connects to a Unitree IM6014, displays live feedback, diagnoses connection/motor problems, and runs short unloaded-bench tests. All speed, position, torque and gain values are **at the output shaft**, with the 38/3 reduction applied by the software.

It uses a [moteus tview-style arrangement](https://mjbots.github.io/moteus/guides/software-installation/#understanding-tview): connection/test controls and a telemetry tree, live plots, and a bottom event/packet console. The IM6014 uses its own binary RS-485 protocol; moteus console/register commands are not interchangeable.

## Start the application

Open this folder in a terminal and run:

```bash
./start_dashboard.sh
```

Or run `python3 -m im6014_bench.dashboard`. An existing installed desktop shortcut can continue to invoke the root `start_dashboard.sh` launcher.

To explore without hardware:

```bash
./start_dashboard.sh --demo
```

Both commands start **disconnected and disarmed**. Demo mode uses simulated motors at IDs 0 and 1 and never opens a real serial adapter. Its yellow **DEMO · no hardware** badge stays visible. Changing source requires disconnecting first.

Install the pinned `requirements.txt` in a Python environment from the repository root. The `im6014_bench` package and `docs/bench/user-manual.md` belong to the checkout. No server, account, or network connection is required for bench operation.

## First connection to this motor

1. Secure the motor housing and leave its output unloaded. Keep a way to isolate motor power within reach. Support every joint if there is anything attached: Stop releases torque.
2. Connect motor power and the supplied USB/RS-485 adapter. A USB connection alone does not power the actuator.
3. Select **Real motor**, then the adapter/channel. For the tested adapter, use **Channel D / interface 03**, currently `/dev/ttyUSB3`. Hover over an adapter entry for its path, serial number and USB location. Click **Refresh** after reconnecting USB.
4. Choose **6 Mbps** and **Motor ID 0** for the tested setup. Motor IDs can be 0–14. The application never broadcasts to ID 15.
5. Click **Connect**. It opens the selected channel and sends **Stop/status**, not a motion command. Confirm live voltage, temperatures and increasing valid-reply count.
6. If the status is **Timeout — disarmed**, support the motor and click **Acknowledge timeout**. The app acknowledges the timeout while in Stop, then immediately restores watchdog protection. It does not reset the motor or clear fault bits.
7. Look for **Ready · Stop** and **No faults or warnings** in Diagnose.

The stable tested device path is:

```text
/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTC2RQ7H-if03-port0
```

Adapter serial: `FTC2RQ7H`, FTDI FT4232H. Linux device numbers can change after reconnecting; prefer the stable by-id path when available. The editable adapter selector also accepts a typed serial-device path.

**Connect, Check, scanning and idle monitoring all send Stop/status packets.** They are active motor communication, not passive listening. Do not use them to observe another program's moving motor. Close the command-line spin script or any other program using the same adapter first.

## Run the 5 rpm test

1. Connect and reach **Ready · Stop**.
2. Select **Speed** and use these values: **5.00 rpm**, **Kd 6.00**, **friction torque 0.160 Nm**, **duration 5.00 seconds**.
3. Check **Motor secured / unloaded — Power cut within reach**.
4. Click **Arm one test**. The motor remains stopped. The permission expires after ten seconds if unused.
5. Click **Run test**. The speed target ramps up and down over 0.75 seconds. Watch measured speed and torque. The blue line is measured speed; the dashed amber line is the requested target.
6. At the end, the motor returns to Stop and the application disarms. Re-arm for another test.

These settings previously produced **5.002 rpm average**, with approximately 4.59–5.48 rpm sampled ripple, on the unloaded motor through the command-line driver. The 0.16 Nm value is bench friction compensation, not a loaded-arm calibration. The dashboard shares the same packet implementation. Those original bench records are retained locally and are not shipped in the public source checkout.

Negative speed selects the opposite direction. Friction compensation is a nonnegative magnitude whose sign follows the speed. Reverse direction and the other dashboard test modes have not been physically validated by that earlier forward-speed run.

The original **5 rpm command cap** and **8 rpm measured-speed trip** were limits chosen for that first bench test. They were not motor ratings. They have been removed. You can enter higher positive or negative speeds; only the signed wire-format range is enforced by the speed input. That range is **not a physical speed rating**. **Measured speed trip** and **Travel trip** are optional, editable per-test checks; **Off** (zero) disables the corresponding check. Both start Off. Higher speed can still trigger the retained torque, voltage, temperature or communication checks. Gains and ramps may need tuning for a different speed or load.

## Position control

1. Connect and reach **Ready · Stop**, then choose **Position**.
2. Select **Relative** to **Move by** an offset from the fresh angle when you click Run. For example, **10 degrees** moves in the positive direction and **−10 degrees** in the reverse direction. Select **Absolute** to **Go to** the current powered-session encoder angle. **Use current position** copies the fresh angle into an absolute target without moving the motor.
3. Set **Target move rate** (a positive value in output rpm), **Position stiffness · Kp** (Nm/rad), **Speed stiffness · Kd** (Nm/(rad/s)) and **Duration**. Initial UI values are 5 rpm, Kp 3, Kd 1, and 5 seconds. These are starting settings for a supported unloaded test, not a tuned loaded-arm controller.
4. Set optional measured-speed/travel trips if wanted. Travel is measured from the starting pose in either direction. A target outside an enabled travel trip is rejected before motion.
5. Check the supported-setup box, **Arm one test**, and **Run test**. The target starts at the measured angle and follows a smooth ramp to the selected angle. The first plot switches to position: blue is measured, dashed amber is the commanded angle.
6. The motor holds the target for the remainder of the duration, then **Stops and disarms**. It does not hold indefinitely. The duration includes movement and settling/holding; choose enough time for both. If the target ramp cannot fit at the selected rate, the request is rejected before FOC.

The move rate limits the rate of the **position target**, not measured shaft speed. Use **Measured speed trip** if you want an actual overspeed trip. The motor uses its built-in proportional position and damping control, with no integral term; position error under friction/load is possible. Stop releases the hold.

Absolute position is a signed multi-turn angle in the current power session. Targets do not wrap to the shortest path. After power loss, the previous multi-turn pose is invalid; read the fresh angle and establish the mechanism's reference before using absolute control. The app does not home or save/restore an arm pose.

## Test modes and limits

| Test | What it does | Inputs | End behavior |
|---|---|---|---|
| Speed | Built-in motor velocity feedback, with output position stiffness zero | Signed output rpm within packet format; Kd 0–6 Nm/(rad/s); friction torque 0–0.5 Nm; 2–10 seconds | Ramp down, Stop, disarm |
| Position | Smooth relative/absolute target, using the motor's position stiffness and damping | Degrees; positive target move rate in rpm; Kp >0–30 Nm/rad; Kd 0–6 Nm/(rad/s); 2–60 seconds | Hold until duration ends, Stop, disarm |
| Torque pulse | Brief feed-forward torque with position/velocity stiffness zero | ±1 Nm; 0.1–0.5 seconds | Stop, disarm |
| Damping | Zero target speed with velocity stiffness; resists manual motion | Kd 0–6 Nm/(rad/s); 2–10 seconds | Stop, disarm |

Start torque tests at **0.1 Nm for 0.25 seconds**. Pure torque does not regulate speed; an unloaded motor can accelerate, so the small time/travel limits matter. A small pulse may not overcome static friction, which is not proof of communication failure. Never raise torque just to force rotation on an unknown mechanism.

For damping, start with **Kd 0.5–1**. Only move the output by hand when the supported, unloaded setup permits it, keeping hands clear of pinch points. Damping is not a position hold or a brake. Only fields relevant to the chosen mode are shown.

Additional measured-feedback limits apply during every test:

- Absolute measured speed: optional **Measured speed trip**; **Off** disables it.
- Absolute measured torque: **4 Nm** maximum.
- Travel from the test's starting pose: **15 degrees for a torque pulse**; optional **Travel trip** for speed, position and damping, with **Off** disabling it.
- Feedback deadline: **50 ms** per reply; during motion, a gap over **100 ms** trips the host watchdog.
- Dashboard heartbeat: a delay over **500 ms** during motion triggers Stop/disarm.
- Voltage: **24–72 V**; driver temperature **−10 to 70 °C**; winding temperature **90 °C** maximum.
- Every FOC packet enables the firmware's approximately **one-second communication watchdog**.

These are conservative application test boundaries, not the actuator's ratings or safety certification. Measured torque is motor telemetry, not an independently measured load-cell value. The application cannot physically stop a motor if communication and power control fail; keep power isolation available.

## Stop, disconnect and recover

Click the red **STOP / DISARM** button or press **Esc** while the application has focus. Stop takes priority over queued motion actions. It cancels the test and invalidates old queued actions. Removing the secured-setup check during an armed/test state also requests Stop.

After operator Stop, click **Check connection** before arming again. After a fault, missing reply, stale state or CRC failure, the app latches disarmed. Healthy feedback returning does not resume the previous test. Fix the problem, re-check while stopped, acknowledge timeout if indicated, and deliberately arm a new test.

**Disconnect** requests Stop and releases the adapter. Closing the window also requests Stop and releases it. If Stop cannot be confirmed, the log says **STOP NOT CONFIRMED**; do not infer that the physical shaft has stopped. Isolate motor power if the situation requires it.

Stop and zero torque can let a gravity-loaded arm fall. This application is for supported bench work, not unattended robot-arm operation.

## Read the dashboard

| Area | Purpose |
|---|---|
| Connection & tests | Source, serial channel, baud, motor ID, Stop/status check, timeout acknowledgment, arm and bounded test settings |
| Telemetry tree | Output speed/position/torque, bus voltage, both temperatures, motor mode, timeout latch and raw fault/warning bitmasks |
| Live plots | Two selectable plots, each showing the last 15 seconds. Choose speed, torque, position, voltage, temperature or reply time |
| Diagnose | Connection state, freshness, Stop confirmation, packet counts, decoded faults, recovery suggestions and discovered IDs |
| Event log & console | Chronological action/rejection/fault messages and a small optional command entry |
| Latest packets | Most recent 20-byte request and CRC-valid 26-byte response in hexadecimal |
| User manual | This manual, available inside the application |

Feedback cards become **—** when data is older than 250 ms. Tree values are greyed when stale. Plots and packet views preserve historical data after a fault; those traces are not current feedback. Check the reply age before drawing conclusions.

The status banner and real/demo badge have explicit dark text on their light backgrounds, so they remain readable with a dark desktop theme. Telemetry text follows the desktop palette. Close and reopen a running instance after updating the application files.

Position is reported in degrees and can be multi-turn while powered. The motor's multi-turn state is not retained through power loss; do not use a previous session's position as the arm's recovered pose. The application does not home or calibrate the motor.

## Discover and select other motors

The dashboard controls **one selected motor at a time** on one serial channel. All motors on that bus must have matching baud and unique IDs.

While connected and disarmed, check the supported-setup box, open **Diagnose**, and click **Discover IDs 0–14 · sends Stop**. Confirm that every motor on the bus is supported. The application sends individual Stop/status requests; it never enables FOC or broadcasts. Motors that do not exist will not reply, so deadline counts rising during a scan are expected. Found IDs appear in Diagnose.

To select another ID, **Disconnect**, change **Motor ID**, and **Connect** again. If the original selected ID does not reply, the adapter can remain open in a fault state for scanning; disconnect and select a discovered ID afterward.

The app does not assign IDs, change stored baud, calibrate encoders, flash firmware, reset motors or clear fault bits. Use the vendor's documented maintenance procedures for those operations after resolving their cause. Coordinated multi-joint control and loaded-arm commissioning require separate integration.

## Diagnose common problems

| Symptom | Checks and next action |
|---|---|
| No adapters in the list | Refresh after USB reconnect; type the known by-id path if needed. Inspect USB cable/port. A missing list entry is different from a motor not replying |
| Permission denied | Confirm access to the serial device; on Linux this is normally the `dialout` group/device permissions. Avoid launching competing controllers |
| Port busy / locked | Close the spin script, vendor assistant or another dashboard window holding that channel, then reconnect |
| Adapter opens, no motor replies | Check motor DC power, Channel D, the selected ID and 4/6 Mbps setting, and the RS-485 cable. The adapter being detected does not prove motor communication |
| Timeout latched, no other faults | Keep motor stopped and supported; click Acknowledge timeout, then verify Ready · Stop before arming |
| CRC failures / deadlines | Inspect cable/adapter and USB connection; disconnect and reconnect while supported. Re-check. The app does not automatically resume motion |
| Overspeed / excessive torque / travel / heat | Leave disarmed. Reduce test settings and inspect mechanical load, wiring and cooling before re-checking |
| Overvoltage / undervoltage | Check supply and regenerative energy handling. Fault decoding reports all set bits; do not clear faults without resolving the cause |
| Speed below target | Small velocity-loop steady-state error is expected without integral control. Adjust the friction compensation cautiously on a supported unloaded bench; do not apply the 0.16 Nm value blindly to a loaded arm |
| Position remains short of target | Check target versus measured position, load/friction, gains and available settling time. Built-in position control has no integral correction. Do not increase stiffness to force an obstructed mechanism |
| Position move needs more time | Increase duration, reduce the angle, or adjust the positive target move rate. The app rejects a ramp that cannot finish within the duration |
| Arm or Run is disabled | Check connection state, timeout/faults, the secured-setup checkbox, and arm expiry. Every test requires a new arm action |
| Stop not confirmed | Feedback is not proof of a stopped shaft when it is missing. Isolate motor power as needed and resolve communication before any new test |

The adapter has previously shown USB control-transfer timeouts on Linux. Normal enumeration alone did not prove usable communication. The successful bench session verified both USB control replies and CRC-valid motor feedback; use the application's valid reply/freshness indicators as the connection evidence.

## Export a debug session

In **Diagnose**, click **Export session · JSON / CSV** and choose a filename.

**JSON** contains retained measurements, settings/state, demo/real flag, fault/link counters, event log and raw validated packets. **CSV** contains the telemetry table for plotting or spreadsheets. Exports preserve the units shown here; CSV measured position and commanded position are radians (`position_rad`, `target_position_rad`) even though the UI displays degrees. Commanded position is blank/null outside position mode.

The app retains the **last 10,000 samples and 500 events** in memory. Diagnose states retained versus total sample count. Export before older samples roll out. Files are saved locally; there is no automatic cloud upload or persistent capture on startup. If a session contains both demo and real connections or multiple IDs, each sample's source/session tag and motor ID distinguish them.

## Optional console

You can use the buttons exclusively. The console supports only these commands:

```text
help
status
stop
check
ack-timeout
arm
speed 5 5
speed -2 3
position 10 5
goto -5 5
torque 0.1 0.25
damping 1 3
```

`speed` takes rpm then seconds. `position` takes a relative offset in degrees then seconds; `goto` takes an absolute angle in degrees then seconds. `torque` takes Nm then seconds. `damping` takes Kd then seconds. Speed uses the current Kd/friction settings. Position commands use the current move rate, Kp and Kd; choose **Position** and tune these first. Console commands use the current optional trip settings. `arm` still requires the secured-setup checkbox. Test commands still require fresh feedback, explicit arm and the same limits as the buttons. There is no shell execution, Python REPL, raw-packet injection or moteus `conf`/`d` command support.

## Validation scope

The previous physical forward-speed test established one motor/adapter's communication and 5 rpm bench operation. Original dashboard validation records remain local. Current offline lifecycle/UI tests are in `tests/bench` and write their result summary to `results/bench`. The earlier physical Stop-only check remains historical evidence. Higher-speed and position changes have been tested offline only. Simulator success is not physical validation of higher speed, position, reverse, torque-pulse, damping, scan-on-a-populated-bus or robot-arm behavior.

For future robot-arm work, establish joint limits, supported failure behavior/brakes, pose recovery and calibration, supply/regeneration handling, load-specific gains, and per-joint timing/freshness before using stiffness or coordinated movement.
