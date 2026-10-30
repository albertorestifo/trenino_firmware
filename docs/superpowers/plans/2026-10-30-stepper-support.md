# Stepper support implementation plan

Implement the supplied generic-axis design in an isolated firmware branch. Application units and calibration remain in the host. First driver is PCA9685/TB6612 with six-state Switec, conventional full-step and half-step commutation; additional driver IDs/modes are rejected until implemented.

1. Add independently testable, strictly sized command/status codecs (IDs 16/17, revision 1). Add malformed-length/version tests before implementation.
2. Add fixed-capacity axis controller with injected driver, integer positions, limits, acceleration, bounded asynchronous homing, immediate stop, release and host watchdog. Prove reset/release/fault reference invalidation, target replacement, two axes and wrapping clocks in native tests.
3. Add direct PCA9685 backend with a single shared shield initialization, explicit ALLCALL/SUBCALL disable and burst updates of the two separate motor ports. Verify wire transactions and fault paths.
4. Integrate serial dispatch, loop servicing and module-reconfiguration shutdown; reject shield pin/address conflicts. Preserve legacy packet definitions and EEPROM format 5.
5. Document exact wire contract, capabilities, retry/reconnect rules and hardware commissioning. Run full native suite and Nano/default hardware compilation; record memory and limitations.

Final design decisions from implementation review: motion config remains volatile;
a separate versioned five-byte explicit shield-address registration at EEPROM500
allows startup shutdown/ALLCALL disable before legacy displays. No blind bus
writes occur. Home includes bounded sweep, optional backoff and total timeout;
status distinguishes an assumed mechanical stop from a host-assigned reference.
