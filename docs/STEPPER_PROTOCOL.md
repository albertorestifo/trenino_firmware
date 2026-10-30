# Configurable stepper axes (firmware 3.1.0, extension revision 1)

The firmware executes electrical motion; the host owns simulator subscriptions,
train-specific behaviour, speed/angle conversion, dial calibration and target
generation. Positions are signed integer **commutation transitions in the selected
stepping mode**, not degrees, revolutions or simulator units. There is no fixed
motor travel, train model, speed scale or gauge geometry in this implementation.

## Implemented capabilities

At most two logical axes, IDs0 and1, on **one** shared PCA9685/TB6612 Adafruit Motor
Shield V2/V3 compatible device. Ports1 (M1+M2) and2 (M3+M4) are independent. Driver
ID1 supports mode1 Switec/X27 six-state, mode2 conventional bipolar two-coil full
step, and mode3 conventional bipolar half step. Other drivers, microstepping,
endstop switches, stall/current sensing and encoders are not implemented. The
driver abstraction owns its phase count and electrical pattern; the motion
engine does not assume six phases.

Only use mode1 for the confirmed white X27.168 LHY1109. Configure travel, safe
speed, acceleration and home sweep from hardware commissioning. The six states
are `09 01 07 06 0e 08`; they are not Adafruit's generic stepper sequence.
Full-step states are `05 06 0a 09`, half-step `01 05 04 06 02 0a 08 09`.
No general-purpose current regulation is provided by this shield: selecting a
generic mode does not make every bipolar motor electrically compatible.

## Framing and discovery

Existing COBS/PacketSerial framing and messages0–15 are unchanged. All multi-byte
numbers below are little endian. Decoded extension packets must have the exact
specified size (maximum41 bytes, below existing64-byte limit). Malformed packets,
unknown revisions and unknown operation codes are ignored and do not refresh the
watchdog. Semantic errors receive a status response.

Identify first. Firmware reports version3.1.0; existing EEPROM module format stays5.
Hosts must gate this extension on version >=3.1.0 and inspect status capability
bytes. Old hosts retain their existing behaviour; they do not gain axis support.

Every command starts with this six-byte header:

| Offset | Type | Meaning |
| --- | --- | --- |
| 0 | u8 | 16 (stepper command) |
| 1 | u8 | 1 (extension revision) |
| 2 | u8 | operation below |
| 3 | u8 | axis0/1; global operations require255 |
| 4 | u16 | request ID; nonzero for mutating axis operations |

| Operation | Value | Size | Payload after header |
| --- | --- | --- | --- |
| Configure | 0 | 41 | configuration below |
| MoveTo | 1 | 10 | i32 absolute target |
| Home | 2 | 6 | none |
| Stop | 3 | 6 | none |
| Enable | 4 | 6 | none |
| Release | 5 | 6 | none |
| GetStatus | 6 | 6 | none |
| KeepAlive | 7 | 6 | none; axis255 |
| Reset | 8 | 6 | none; axis255 |
| Reference | 9 | 10 | i32 host-assigned position |
| ForgetDriver | 10 | 6 | none; axis255 |

Configure payload (offsets from start of packet):

| Offset | Type | Meaning / accepted values |
| --- | --- | --- |
| 6 | u8 | driver=1 |
| 7 | u8 | mode=1 (Switec six-state),2 (full),3 (half) |
| 8 | u8 | shield seven-bit address0x60–0x77; default hardware0x60 |
| 9 | u8 | shield stepper port1 or2 |
| 10 | u8 | reverse electrical direction,0/1 |
| 11 | u8 | idle behaviour:0 release,1 hold |
| 12 | u8 | host-loss behaviour:0 release,1 hold last electrical phase |
| 13 | u8 | referencing:0 host Reference only,1 bounded internal-stop Home |
| 14 | i32 | minimum position, >=-1000000 |
| 18 | i32 | maximum position, <=1000000, strictly greater than minimum |
| 22 | u16 | maximum speed,1–200 transitions/s |
| 24 | u16 | acceleration,1–10000 transitions/s² |
| 26 | i8 | home logical direction,-1 or+1 (strategy1) |
| 27 | u16 | home sweep transitions,1–65535 (strategy1) |
| 29 | u16 | home speed,1–max_speed (strategy1) |
| 31 | u16 | total Home timeout,1–60000ms (strategy1) |
| 33 | i32 | position assigned after sweep and backoff; inside limits |
| 37 | u16 | host watchdog,100–60000ms; cannot be disabled |
| 39 | u16 | backoff transitions,0–home_steps-1 (strategy1) |

Home parameters other than home_position are ignored for strategy0. Choose a
timeout longer than `(sweep+backoff)/home_speed`, allowing I2C and loop jitter.
The configured speed is a ceiling, not a promised real-time throughput.

## State and command semantics

Configure validates before changing an existing axis. Once valid, it releases the
old port, replaces config and starts disabled with unknown position. Two axes may
share the same address but must use distinct ports. A second shield is rejected.
Configure does not move or automatically Home. Invalid configs preserve the old
axis; I2C failures invalidate the shared shield's axes. Changing a registered
shield address requires Reset, ForgetDriver, then Configure at the new address.

MoveTo requires a known reference. Latest target replaces the previous target;
there is no queue. The planner ramps in the configured units, brakes before
reversal and approaches the new target within soft limits. A target at the
current position while moving brakes before returning. Invalid targets are
rejected, never silently clamped. Estimates are open-loop and cannot detect lost
steps or mechanical obstruction.

Home is bounded and asynchronous. It sends exactly home_steps transitions toward
the assumed internal stop at home_speed, then home_backoff in the opposite
direction, then assigns home_position. Soft position limits do not constrain the
unreferenced sweep. The total timeout bounds both stages. The host must explicitly
opt into strategy1 and commission a sweep sufficient to reach its real stop.
**Completion means an assumed internal-stop reference, not detection of a stop.**
The firmware has no feedback to establish that the shaft moved or contacted it.
Home while moving/homing is rejected. MoveTo during Home is rejected.

Reference is permitted only for strategy0 and a stationary axis. It energizes the
current phase then trusts the supplied coordinate; the host must establish the
physical reference. Enable energizes the current phase and never creates a
reference. Release disables the bridge and forgets the reference. Stop immediately
halts command generation, holds/releases according to idle policy and invalidates
a moving/homing reference because the rotor may coast. It is not a decelerating
stop. Reset attempts release on both ports and discards volatile axis configs and request
history; it does not erase shield registration or legacy module configuration.
If any release write fails, Reset reports I2C error while still discarding the
volatile configs/reference; output power state is then unconfirmed.

Idle hold=0 **forgets the reference on every completion**, including Home. It is
appropriate only when the host explicitly re-references before further motion;
continuous absolute positioning requires hold=1. A fault requires Configure to
clear; merely enabling or sending KeepAlive cannot recover a reference.

KeepAlive refreshes the shared host lease. Successfully accepted new mutating
axis commands also refresh it. GetStatus, legacy traffic, outgoing heartbeat,
malformed packets, rejected commands and duplicate retries do not. Send KeepAlive
well within the smallest configured watchdog (for example every250ms for1000ms).
On expiry all configured axes fault and forget their references. Policy0 attempts
release, policy1 keeps the last phase energized. Neither preserves a trustworthy
reference after an emergency halt. Any I2C write failure faults both axes on the
shared shield; release is attempted but cannot be guaranteed on a broken bus.

Each accepted command gets an immediate status: `result=0` acknowledges acceptance,
**not physical completion**. Poll status or observe asynchronous state changes
to determine completion. Each axis remembers only its most recent mutating
request ID, operation and result. Retrying that same ID+operation returns the
cached result/current status and does not execute again (especially Home).
Never reuse an ID for different content. Retry only the latest operation; after
any intervening mutation the earlier ID is no longer protected. Before16-bit ID
reuse, start a new session with Reset. Home is never automatically replayed by
firmware or the reconnect procedure.

Reconnect sequence: Identity; query capabilities; Reset; Configure each axis;
explicit Home or Reference; then MoveTo. Reset on reconnect is intentional even
if the board did not reboot. A reboot always loses axis config and reference.

An axis-only device can retain the host deployment identity without configuring
dummy inputs or displays. Before axis Configure, deploy the empty legacy module
list with the exact8-byte packet `02 <config_id:u32LE> 00 00 ff`. It is acknowledged
with ConfigurationStored and the config_id survives reboot with zero legacy
modules. It supersedes any unfinished multipart deployment. This is available
from firmware3.1.0; earlier firmware rejects it. Like other completed legacy
deployments it removes old module instances and resets/releases all axes, but
preserves shield registration. It does not clear independent digital SetOutput
states or blank removed displays. Complete deployment and retries before axis
configuration/referencing; do not replay it after starting motion.

## Status packet (type17, exactly23 bytes)

| Offset | Type | Meaning |
| --- | --- | --- |
| 0,1 | u8,u8 | 17, revision1 |
| 2,3 | u8,u8 | operation and axis echoed; async uses GetStatus and request0 |
| 4 | u16 | request ID |
| 6 | u8 | result code |
| 7 | u8 | state:0 disabled,1 idle,2 moving,3 homing,4 fault |
| 8 | u8 | bit0 configured,bit1 known,bit2 energized; bits3–4 reference kind:0 unknown,1 assumed stop,2 host assigned |
| 9 | u8 | fault:0 none,1 driver I2C,2 home timeout,3 host lost |
| 10 | i32 | estimated position; invalid when known bit is clear |
| 14 | i32 | latest target; invalid until referenced |
| 18 | u8 | electrical phase index in configured mode |
| 19 | u8 | max axes=2 |
| 20 | u8 | driver bitset:bit0 ID1 implemented |
| 21 | u8 | mode bitset:bit0 six-state,bit1 full-step,bit2 half-step |
| 22 | u8 | homing bitset:bit0 bounded internal-stop implemented; host Reference always available |

Results:0 OK,1 invalid value,2 bad axis,3 unsupported driver/mode/strategy,
4 resource conflict,5 unconfigured,6 unreferenced,7 target outside limits,
8 busy,9 I2C/fault error. Global responses use axis255 and no axis data.
Asynchronous status is emitted on state/reference/fault changes, not every step.

## Persistence, startup and resource ownership

Axis motion configs are volatile; legacy EEPROM format5 is preserved. A separate
five-byte record at EEPROM500..504 contains `53 54 01 address ~address`. Version1
and complement validation prevent erased/stale data becoming an arbitrary I2C
address. It records **only** the explicitly configured shield address, not motion
settings or a reference. A successful Configure registers that address; repeating
it does not rewrite EEPROM. The commit marker is written last. ForgetDriver
requires no configured axes and successfully shuts down the registered device
before invalidating the record. Run Reset first.

Before legacy modules initialize at boot, registered hardware is shut down and
PCA9685 ALLCALL/SUBCALL disabled by unicast. No address scan writes and no general
call reset occur. A failed registered shutdown defers all EEPROM module init and
reports ModuleError(init_failed) on Identity. Recover the bus, Configure, and
reapply legacy module config. Registered SDA/SCL pins cannot be reused by analog,
button, matrix or SetOutput operations. Display address collisions are rejected.
Legacy module reconfiguration releases axes and requires a fresh axis Configure.

An unregistered PCA9685 powers up responding to ALLCALL0x70, which can collide
with an HT16K33 at0x70 **before firmware knows it exists**. First commissioning:
connect the shield without a0x70 display, Configure an axis to register it, then
connect/configure the display. Recommission with this order after erasing EEPROM,
changing shield address or replacing hardware. A firmware-only solution cannot
identify an unknown0x70 responder safely. Do not run older firmware with a shield
and0x70 display; older firmware does not consult this registration.

Each motor write is one25-byte I2C burst (within AVR Wire32-byte buffer), changing
only its own six PCA channels. The mode disables ALLCALL and all SUBCALLs and
enables auto-increment, with output updates at STOP. Both bridge PWM channels use
full-on/full-off; no PWM frequency/duty configuration or microstepping is claimed.
No other use of the shield's motors/servo channels is coordinated here.

## Wiring and verification limits

For mode1 connect physical X27 contacts1,2 to the first bridge outputs and3,4 to
the second, preserving order. Logical bits0..3 are contacts1..4. Shield port1 maps
PCA channels PWMA8,AIN2=9,AIN1=10,BIN1=11,BIN2=12,PWMB13; port2 uses2,3,4,5,6,7.
Verify contact numbering and winding pairs on the actual motor before power.
Reverse is configurable; do not infer viewing-direction from an untested clone.

Scheduler services at most one transition per axis per update, uses wrapping
32-bit microsecond subtraction and never bursts overdue steps. I2C, display
reinitialization/retry delays and serial output introduce jitter and can lower
achieved speed. AVR Wire transactions time out after25ms each. This is cooperative
control, not interrupt-driven real-time pulse generation. The firmware services
motion before module scans and between outgoing sensor readings; long legacy
operations remain bounded by their existing bus/retry behaviour.

Native tests and Nano compilation validate software behaviour and wire bytes;
there is no hardware validation here. Commission polarity, winding mapping,
voltage/current, acceleration, travel, home direction/sweep/backoff and pointer
load conservatively before attaching a delicate needle or relying on a reference.

The Nano build currently uses1339 bytes of **static** SRAM out of2048; the remaining
709 bytes are shared by heap, stack and interrupts, not reserved axis capacity.
AVR object sizes measured with the target compiler are: heartbeat10 bytes,
HT16K33 module24, analog12, button9, matrix231, plus allocator overhead (typically
2 bytes per allocation). Two axes are already included in static usage. For the
intended two axes plus one display, heartbeat/display allocations consume about
38 further bytes, leaving about671 for stack/interrupts. This is an estimate,
not a measured runtime high-water mark. Existing PacketSerial decoding uses a
variable stack buffer (up to255 bytes), legacy sendMessage uses128 bytes, and
call frames/interrupts also need space. The global eight-module limit does **not**
guarantee eight arbitrary modules fit a Nano; multiple231-byte matrices clearly
do not. No maximum-module stress or on-device stack/heap measurement was possible.
Use a larger board for complex configurations and measure runtime headroom on
the actual configuration before deployment.

Electrical references: [Adafruit shield pin mapping](https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library/blob/master/Adafruit_MotorShield.cpp),
[NXP PCA9685 datasheet](https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf), and
[SwitecX25 six-state source](https://github.com/clearwater/SwitecX25/blob/master/SwitecX25.cpp).
