#pragma once
#include <Arduino.h>

#include <atomic>
#include <cstdint>

/**
 * @brief Console module for user interaction over UART using ESP32Console.
 *
 * Contracts & Assumptions
 * - Initializes once at startup (allowed to allocate during setup only).
 * - Runs fully asynchronous in its own task (provided by ESP32Console).
 * - NOT real-time critical; designed to avoid interfering with control loops.
 * - Do not use Serial concurrently (ESP32Console owns stdio).
 *
 * Thread-safety
 * - Public setters use atomics; command handlers are short and non-blocking.
 *
 * Error handling
 * - Functions return bool for success; command handlers return
 * EXIT_SUCCESS/FAILURE.
 */
namespace cnc::console {

enum class BaudRate : uint32_t {
  BR_115200 = 115200U,
  BR_230400 = 230400U,
  BR_460800 = 460800U,
  BR_921600 = 921600U
};

struct ConsoleConfig final {
  BaudRate baud{BaudRate::BR_115200};
  const char* prompt{"CNC> "};              // shown before begin()
  int8_t rx_pin{-1};                        // -1 -> default
  int8_t tx_pin{-1};                        // -1 -> default
  bool register_builtin_system_cmds{true};  // meminfo, sysinfo, reboot, help...
};

class UserConsole final {
 public:
  // Singleton accessor (SOLID: single responsibility for the user I/O shell)
  static UserConsole& Instance();

  // RAII lifecycle (call Begin() from setup/initialization code)
  bool Begin(const ConsoleConfig& cfg) noexcept;
  void End() noexcept;

  // Optional: provide a fast status line (shown by `status`)
  // The function must be short, non-blocking, and not allocate.
  using StatusFn =
      const char* (*)();  // returns pointer to static/constexpr text
  void SetStatusProvider(StatusFn fn) noexcept;

  // NEW: Register a custom command (name, handler, help). Returns false if the
  // console is not started.
  bool RegisterCommand(const char* name, int (*handler)(int, char**),
                       const char* help) noexcept;

  // Verbose flag can be polled by other modules (atomic, thread-safe)
  bool IsVerbose() const noexcept { return verbose_.load(); }

  // --- Non-copyable / non-movable (single hardware console) ---
  UserConsole(const UserConsole&) = delete;
  UserConsole& operator=(const UserConsole&) = delete;
  UserConsole(UserConsole&&) = delete;
  UserConsole& operator=(UserConsole&&) = delete;

 private:
  UserConsole() = default;
  ~UserConsole() = default;

  // Command handlers (static to satisfy ESP32Console’s C-style callback)
  static int CmdStatus(int argc, char** argv);
  static int CmdEcho(int argc, char** argv);
  static int CmdVerbose(int argc, char** argv);

  // Helpers
  static bool ParseBooleanToken(const char* token, bool& out) noexcept;

 private:
  std::atomic<bool> started_{false};
  std::atomic<bool> verbose_{false};
  StatusFn status_provider_{nullptr};
};

}  // namespace cnc::console
