# Design Notes & Compliance (English)

**Parallax-style edge capture:** Capture both rising and falling edges; compute t_on and t_off with a minimal ISR and perform all math in a task context.

**MISRA C++ mindset:** Clear ownership and lifetimes, no undefined behavior, explicit casts and widths, no hidden allocations in real-time paths.

**RAII:** Initialize and release GPIO/ISR resources in constructors/destructors; idempotent init/deinit.

**constexpr / inline:** Keep constants and pure conversions at compile time; keep small helpers inline.

**SOLID:**

**SRP:** Mae3Encoder handles edge capture & conversion; EncoderManager coordinates channels; observers consume positions.

**OCP:** Add new observers without touching core logic.

**LSP/ISP/DIP:** Depend on IEncoderObserver interface; no hard coupling to concrete sinks.

**Enums over macros:** Use enum class for IDs/status/levels; no magic numbers.

**No dynamic allocation in RT:** Use static storage, in-place construction, and fixed capacities.

**Design patterns:** Singleton manager, factory-style construction of channels, Observer for updates.

**Non-blocking APIs:** tryGetPosition() and pollAndNotify() never block; publish only when a fresh sample exists.

**Interrupts kept tiny:** ISR only timestamps and stores deltas; no logging/allocations/queues in ISR.

**Watchdog:** Enable and periodically reset from the polling task.

**Thread-safety:** Use short critical sections (spinlock/portMUX) between ISR and task; atomics for cached values.

**FreeRTOS style:** Static task creation, explicit priorities, no busy waiting.

**Modular/layered:** Domain types → driver (Mae3Encoder) → manager (activation of 1/4 encoders) → app (observers).

**Unit tests:** Isolate the duty-cycle → position math and edge-case behavior; test on host (native) without hardware.

**API contract:** Call configure() then setActive(); only one encoder enabled at any time; output range is [0..4095] and robust to 0↔100% wrap and period drift within spec.


# Notes for Arduino/ESP32:

**ISR minimal:** Only use digitalRead + micros() and store deltas—no printing, calculations, or heap usage inside the ISR.

**No dynamic allocation:** Everything is static, on the stack, or in fixed arrays.

**SOLID / RAII / Enums:** Principles are applied; channels are enabled/disabled through the Manager.

**Non-blocking:** Position is computed only when a fresh sample is available.

**Thread-safety:** On Arduino, short critical sections use noInterrupts() / interrupts().

**Unit tests:** In the env:native environment only the math conversion is tested, with no Arduino dependencies.


Why a Shared ISR Works Safely

All encoder channels register the same ISR function (Mae3Encoder::isrShared).
Each ISR call receives a pointer to its own channel object. The ISR only updates the state of that specific object (its own EdgeCapture), so there is no shared data and therefore no race between channels.

By design, only one encoder is active at a time:

The EncoderManager disables all other channels before enabling the next one.

If an edge arrives on a disabled pin, the ISR exits immediately with an early return.

Inside the ISR there is no printing, no math, no heap allocation. It does the minimum work:

digitalRead

micros()

store the delta

This keeps ISR execution time minimal and prevents timing conflicts.

If, in the future, multiple encoders are enabled at once, the same design still holds:

each channel has independent state

the ISR only touches its own data

only the Observer/consumers need to be prepared for multiple sources.

Design Compliance

MISRA C++ mindset: fixed-width types, early returns, no undefined behavior, no dynamic allocation in real-time paths.

RAII: clear init/deinit ownership, idempotent setup, objects live on stack or in static storage.

constexpr / inline: constants and tiny helpers resolved at compile-time.

SOLID principles:

Single Responsibility: Mae3Encoder (edge capture), EncoderManager (coordination), IEncoderObserver (consumption).

Open/Closed: observers can be added without touching core logic.

LSP/ISP/DIP: depend on the IEncoderObserver interface, not concrete classes.

Enums over macros: enum class used for IDs and states (no magic numbers).

No dynamic allocation in RT path: static storage, fixed capacities.

Design patterns: Manager as lightweight factory, Observer for event delivery.

Non-blocking APIs: tryGetPosition() and pollAndNotify() never block.

Minimal ISR: only pin read + timestamp + local state update.

Thread-safety: short critical sections with noInterrupts()/interrupts(), no cross-channel shared state.

Unit tests: conversion math and edge cases tested in env:native, no Arduino dependency.

Layered structure:

Domain types → Driver (Mae3Encoder) → Manager (N channels) → Application (observers).