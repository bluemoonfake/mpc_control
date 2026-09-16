# HITL system tests

HITL starts only after the equivalent SITL gate passes. It verifies the same
controller binary on the target flight controller/companion-computer transport,
including DDS latency, scheduling, link loss, RC takeover and PX4 failsafes.

Running this specification requires an operator checklist and explicit hardware
preparation; it is not executed automatically by the build.
