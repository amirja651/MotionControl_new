#include "UserConsole.hpp"
#include <ESP32Console.h>  // Library: jbtronics/ESP32Console
#include <cstdio>

using namespace ESP32Console; // per library README usage
// See: include <ESP32Console.h>, Console console; registerCommand(); begin(); etc. :contentReference[oaicite:2]{index=2}

namespace cnc::console {

namespace {
    // The underlying ESP32Console object lives here (one instance).
    Console g_console;
    // Convenience: normalized EXIT codes
    constexpr int OK = EXIT_SUCCESS;
    constexpr int ERR = EXIT_FAILURE;
}

// ---------- Public API ----------
UserConsole& UserConsole::Instance() {
    static UserConsole instance;
    return instance;
}

bool UserConsole::Begin(const ConsoleConfig& cfg) noexcept {
    if (started_.load()) {
        return false;
    }

    // Configure prompt BEFORE begin() (per library docs)
    g_console.setPrompt(cfg.prompt); // allowed pre-begin. :contentReference[oaicite:3]{index=3}

    // Start console: takes ownership of stdio/UART and spawns its own task
    // Note: you can set RX/TX pins similar to Serial.begin()
    const uint32_t baud = static_cast<uint32_t>(cfg.baud);
    if (cfg.rx_pin >= 0 && cfg.tx_pin >= 0) {
        g_console.begin(baud, cfg.rx_pin, cfg.tx_pin);
    } else {
        g_console.begin(baud);
    }

    if (cfg.register_builtin_system_cmds) {
        g_console.registerSystemCommands(); // help, reboot, sysinfo, meminfo, etc. :contentReference[oaicite:4]{index=4}
    }

    // Register project-specific commands (short, non-blocking)
    g_console.registerCommand(ConsoleCommand("status", &UserConsole::CmdStatus,
        "Show brief CNC status and resources."));
    g_console.registerCommand(ConsoleCommand("echo", &UserConsole::CmdEcho,
        "Echo arguments back (usage: echo <text...>)"));
    g_console.registerCommand(ConsoleCommand("verbose", &UserConsole::CmdVerbose,
        "Set verbosity (usage: verbose <0|1|on|off|true|false>)"));

    started_.store(true);
    // Friendly banner (printf is recommended with ESP32Console) :contentReference[oaicite:5]{index=5}
    std::printf("\nConsole ready. Type 'help' to list commands.\n");
    return true;
}

void UserConsole::End() noexcept {
    // ESP32Console has no explicit end(); leaving as no-op for now.
    started_.store(false);
}

void UserConsole::SetStatusProvider(StatusFn fn) noexcept {
    status_provider_ = fn;
}

// ---------- Helpers ----------
bool UserConsole::ParseBooleanToken(const char* token, bool& out) noexcept {
    if (token == nullptr) { return false; }
    // Lowercase, minimal heapless parsing
    // Accepted: 1/0, on/off, true/false, enable/disable, yes/no
    const struct KV { const char* key; bool val; } table[] = {
        {"1", true}, {"0", false}, {"on", true}, {"off", false},
        {"true", true}, {"false", false}, {"enable", true}, {"disable", false},
        {"yes", true}, {"no", false}
    };
    for (const auto& kv : table) {
        // strcasecmp is available; if not, implement a simple case-insensitive compare
        if (::strcasecmp(token, kv.key) == 0) {
            out = kv.val;
            return true;
        }
    }
    return false;
}

// ---------- Command callbacks ----------
int UserConsole::CmdStatus(int argc, char** /*argv*/) {
    (void)argc;
    // Uptime and heap are safe to query here; avoid long prints
    const uint32_t ms = millis();
    const uint32_t s = ms / 1000U;
    const uint32_t free_heap = ESP.getFreeHeap();

    std::printf("uptime=%lus free_heap=%u bytes verbose=%s\n",
                static_cast<unsigned long>(s),
                static_cast<unsigned>(free_heap),
                Instance().verbose_.load() ? "on" : "off");

    if (Instance().status_provider_ != nullptr) {
        const char* line = Instance().status_provider_();
        if (line != nullptr) {
            std::printf("%s\n", line);
        }
    }
    return OK;
}

int UserConsole::CmdEcho(int argc, char** argv) {
    if (argc <= 1) {
        std::printf("usage: echo <text...>\n");
        return ERR;
    }
    for (int i = 1; i < argc; ++i) {
        std::printf("%s%s", argv[i], (i + 1 < argc) ? " " : "\n");
    }
    return OK;
}

int UserConsole::CmdVerbose(int argc, char** argv) {
    if (argc != 2) {
        std::printf("usage: verbose <0|1|on|off|true|false>\n");
        return ERR;
    }
    bool v = false;
    if (!ParseBooleanToken(argv[1], v)) {
        std::printf("error: invalid boolean value '%s'\n", argv[1]);
        return ERR;
    }
    Instance().verbose_.store(v);
    std::printf("verbose=%s\n", v ? "on" : "off");
    return OK;
}

} // namespace cnc::console
