#pragma once
#include <Arduino.h>

#include <atomic>
#include <cstdint>

extern "C"
{
#include "driver/gpio.h"  // ESP-IDF GPIO ISR service
#include "esp_intr_alloc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "soc/gpio_reg.h"
}

/// \file mae3_encoder.hpp
/// \brief MAE3 PWM encoder capture (header-only, ISR-light, no heap in RT
/// paths)
///
/// Contracts & Assumptions
/// - Initialization allocates/configures resources; no dynamic allocation afterwards.
/// - ISR path is minimal: only timestamp deltas and volatile stores (no math/printf).
/// - One active encoder channel at a time (per original design).
/// - Observer callbacks must be short and non-blocking; they run from a task context.
/// - For thread-safety: task-side critical sections use FreeRTOS portMUX spinlocks.
/// - If cycle counter timing is enabled, CPU frequency is sampled at init (best-effort).
///
/// Compliance highlights
/// - MISRA C++-friendly style, RAII where applicable, no blocking in ISRs.
/// - `enum class` instead of macros for logical constants; configuration via `constexpr`.
/// - Avoids `noInterrupts()/interrupts()`; uses FreeRTOS critical sections.
/// - No dynamic allocation at runtime (fixed-size arrays, atomics).

namespace mae3
{
    // ------------------------------ Fast GPIO read ---------------------------------
    inline bool fastRead(int pin) noexcept
    {
        if (pin < 32)
        {
            return (REG_READ(GPIO_IN_REG) >> pin) & 0x1U;
        }
        return (REG_READ(GPIO_IN1_REG) >> (pin - 32)) & 0x1U;
    }

    // ------------------------------ Time base (ticks) ------------------------------
    struct TimeStamp final
    {
        // Init must be called once during system setup (EncoderManager::configure does this).
        static inline void init() noexcept
        {
            const uint32_t mhz = getCpuFrequencyMhz();
            cycles_per_us_.store((mhz == 0U) ? 240U : mhz, std::memory_order_relaxed);
        }

        static inline uint32_t nowTicks() noexcept
        {
            return ESP.getCycleCount();  // 32-bit wrap is fine for deltas
        }

        static inline std::double_t toMicros(uint32_t v) noexcept
        {
            const uint32_t      c           = cycles_per_us_.load(std::memory_order_relaxed);
            const std::uint32_t cyclesPerUs = c == 0U ? 240U : c;                            // If cycles_per_us_ has not been initialized yet, fall back to 240 (ESP32 @ 240 MHz)
            return static_cast<std::double_t>(v) / static_cast<std::double_t>(cyclesPerUs);  // Convert CPU cycles (ticks) into microseconds with fractional precision
        }

    private:
        static inline std::atomic<uint32_t> cycles_per_us_{240U};
    };

    // ----------------------------- Types & constants -------------------------------
    enum class Status : std::uint8_t
    {
        Ok = 0U,
        InvalidArg,
        NotInitialized,
        AlreadyInitialized
    };

    enum class LogicLevel : std::uint8_t
    {
        Low  = 0U,
        High = 1U
    };

    struct Constants final
    {
        static constexpr std::uint32_t kResolutionBits     = 12U;
        static constexpr std::uint32_t kResolutionSteps    = (1UL << kResolutionBits);                           // 4096
        static constexpr std::double_t kMaxResolutionSteps = static_cast<std::double_t>(kResolutionSteps + 2U);  // 4098.0f
        static constexpr std::double_t kMaxIndex           = static_cast<std::double_t>(kResolutionSteps - 1U);  // 4095.0f
        static constexpr std::double_t kBorderIndex        = static_cast<std::double_t>(kResolutionSteps - 2U);  // 4094.0f

        // Valid PWM frame bounds for MAE3 (typical ~4.0ms period). Keep generous margins.
        static constexpr std::double_t kMinTonUs   = 0.95f;
        static constexpr std::double_t kMaxTonUs   = 1.05f;
        static constexpr std::double_t kMinCycleUs = 3892.0f;  // ~3.9 ms
        static constexpr std::double_t kMaxCycleUs = 4302.0f;  // ~4.3 ms
        static constexpr std::double_t kZeroZoneUs = 8000.0f;
    };

    struct GpioConfig final
    {
        int  pin;
        bool pullup;
        bool pulldown;  // Not relied upon by core logic; prefer external pulldown if needed
        bool inverted;
    };

    class IEncoderObserver
    {
    public:
        virtual ~IEncoderObserver() = default;
        /// \brief Called when a new position sample is available.
        /// \param index Logical encoder index
        /// \param position 0..4095 (12-bit)
        /// \param tonUs_f High-time (µs)
        /// \param toffUs_f Low-time (µs)
        virtual void onPositionUpdate(std::uint8_t index, std::double_t position, std::double_t tonUs_f, std::double_t toffUs_f) = 0;
    };

    // -------------------------- EdgeCapture (per channel) --------------------------
    class EdgeCapture
    {
    public:
        EdgeCapture() = default;

        // Task-side: reset capture state
        void reset(std::uint32_t now_ticks) noexcept
        {
            portENTER_CRITICAL(&s_mux_);
            last_edge_ticks_ = now_ticks;
            ton_ticks_       = 0U;
            toff_ticks_      = 0U;
            high_valid_      = false;
            low_valid_       = false;
            ready_           = false;
            portEXIT_CRITICAL(&s_mux_);
        }

        // ISR-side: store delta only (no math/log/heap)
        inline void onEdgeISR(bool level, std::uint32_t now_ticks) noexcept
        {
            const std::uint32_t dt = now_ticks - last_edge_ticks_;
            last_edge_ticks_       = now_ticks;
            if (level)
            {
                toff_ticks_ = dt;
                low_valid_  = true;
            }
            else
            {
                ton_ticks_  = dt;
                high_valid_ = true;
            }
            if (high_valid_ && low_valid_)
            {
                ready_      = true;
                high_valid_ = false;
                low_valid_  = false;
            }
        }

        // Task-side: consume most recent full sample (non-blocking)
        bool tryConsume(std::uint32_t& ton_ticks, std::uint32_t& toff_ticks) noexcept
        {
            portENTER_CRITICAL(&s_mux_);
            const bool rdy = ready_;
            if (rdy)
            {
                ton_ticks  = ton_ticks_;
                toff_ticks = toff_ticks_;
                ready_     = false;
            }
            portEXIT_CRITICAL(&s_mux_);
            return rdy;
        }

    private:
        // NOTE: These fields are touched from ISR; keep them trivially copyable and
        // small.
        volatile std::uint32_t last_edge_ticks_{0U};
        volatile std::uint32_t ton_ticks_{0U};
        volatile std::uint32_t toff_ticks_{0U};
        volatile bool          high_valid_{false};
        volatile bool          low_valid_{false};
        volatile bool          ready_{false};

        // Task-side critical section (NOT used in ISR)
        static inline portMUX_TYPE s_mux_ = portMUX_INITIALIZER_UNLOCKED;
    };

    // ----------------------------- Mae3Encoder (per channel)
    // -----------------------------
    class Mae3Encoder
    {
    public:
        Mae3Encoder() = default;
        Mae3Encoder(std::uint8_t index, const GpioConfig& cfg) : index_{index}, cfg_{cfg}, _inAbsenceOfInterruptFlag(false) {}

        Status init()
        {
            if (inited_)
            {
                return Status::AlreadyInitialized;
            }
            pinMode(cfg_.pin, cfg_.pullup ? INPUT_PULLUP : INPUT);
            TimeStamp::init();  // Ensure time base initialized
            capture_.reset(TimeStamp::nowTicks());
            // Attach shared ISR using "this" as argument
            attachInterruptArg(digitalPinToInterrupt(cfg_.pin), &Mae3Encoder::isrShared, this, CHANGE);
            disable();  // keep disabled until selected active
            inited_ = true;
            return Status::Ok;
        }

        Status deinit()
        {
            if (!inited_)
            {
                return Status::NotInitialized;
            }
            detachInterrupt(digitalPinToInterrupt(cfg_.pin));
            inited_  = false;
            enabled_ = false;
            return Status::Ok;
        }

        Status enable()
        {
            if (!inited_)
            {
                return Status::NotInitialized;
            }
            enabled_ = true;
            return Status::Ok;
        }

        Status disable()
        {
            if (!inited_)
            {
                return Status::NotInitialized;
            }
            enabled_ = false;
            return Status::Ok;
        }

        /// \brief Fetch last complete sample; returns false if nothing new.
        bool tryGetPosition(std::double_t& pos_out_f, std::double_t& tonUs_f, std::double_t& toffUs_f) noexcept
        {
            std::uint32_t ton_raw{0U}, toff_raw{0U};
            if (!capture_.tryConsume(ton_raw, toff_raw))
            {
                return false;
            }

            std::double_t ton_us_f    = TimeStamp::toMicros(ton_raw);
            std::double_t toff_us_f   = TimeStamp::toMicros(toff_raw);
            _inAbsenceOfInterruptFlag = true;

            if (cfg_.inverted)
            {
                const std::double_t tmp_f = ton_us_f;
                ton_us_f                  = toff_us_f;
                toff_us_f                 = tmp_f;
            }

            pos_out_f = dutyToPosition(ton_us_f, toff_us_f);
            tonUs_f   = ton_us_f;
            toffUs_f  = toff_us_f;
            return true;
        }

        inline std::uint8_t index() const noexcept
        {
            return index_;
        }

        void attachInAbsenceOfInterrupt(void (*callback)())
        {
            _inAbsenceOfInterrupt = callback;
        }
        void handleInAbsenceOfInterrupt()
        {
            if (_inAbsenceOfInterruptFlag)
            {
                _inAbsenceOfInterruptFlag = false;
                if (_inAbsenceOfInterrupt)
                    _inAbsenceOfInterrupt();
            }
        }
        void setInAbsenceOfInterruptFlag(bool flag)
        {
            _inAbsenceOfInterruptFlag = flag;
        }

    private:
        static void IRAM_ATTR isrShared(void* arg)
        {
            auto* self = static_cast<Mae3Encoder*>(arg);
            if ((self != nullptr) && self->enabled_)
            {
                // We trust hardware to supply CHANGE edges; read current level and
                // timestamp
                const bool level = (fastRead(self->cfg_.pin) != 0);
                self->capture_.onEdgeISR(level, TimeStamp::nowTicks());
            }
        }

        // Convert PWM high/low durations (µs) to 12-bit position (0..4095)
        static std::double_t dutyToPosition(std::double_t ton_us_f, std::double_t toff_us_f) noexcept
        {
            const std::double_t tcycle_f = ton_us_f + toff_us_f;

            if (tcycle_f > Constants::kZeroZoneUs || ton_us_f < Constants::kMinTonUs)
            {
                // Zero-range -> return 0 value
                last_valid_pos_.store(0.0f, std::memory_order_relaxed);
                return 0.0f;
            }
            if ((tcycle_f < Constants::kMinCycleUs) || (tcycle_f > Constants::kMaxCycleUs))
            {
                // Out-of-range period -> return last valid value
                return last_valid_pos_.load(std::memory_order_relaxed);
            }

            const std::double_t num_f = ton_us_f * Constants::kMaxResolutionSteps;  // Numbers in this range are safe in 32 bits

            std::double_t x_f = (num_f / tcycle_f);  // floor
            x_f               = (x_f > 0.0f) ? (x_f - 1.0f) : 0.0f;

            std::double_t pos_f;
            if (x_f <= Constants::kBorderIndex)
                pos_f = x_f;
            else                               // x >= 4095.0f
                pos_f = Constants::kMaxIndex;  // End clip according to datasheet

            last_valid_pos_.store(pos_f, std::memory_order_relaxed);
            Serial.print("pos_f: ");
            Serial.println(pos_f);
            return pos_f;
        }

        static inline std::atomic<std::double_t> last_valid_pos_{0.0f};

        std::uint8_t  index_{0U};
        GpioConfig    cfg_{};
        bool          inited_{false};
        volatile bool enabled_{false};
        EdgeCapture   capture_{};

        // Optional movement complete callback
        volatile bool _inAbsenceOfInterruptFlag;
        void (*_inAbsenceOfInterrupt)();
    };

    // ---------------- Manager (templated N; fixed capacity; one-active-at-a-time)
    // ----------------
    template <std::size_t N> class EncoderManager
    {
    public:
        Status configure(const GpioConfig (&pins)[N])
        {
            if (inited_)
            {
                return Status::AlreadyInitialized;
            }
            TimeStamp::init();  // Initialize time base once
            for (std::size_t i = 0U; i < N; ++i)
            {
                encs_[i] = Mae3Encoder(static_cast<std::uint8_t>(i),
                                       pins[i]);  // RAII: stack object
                (void)encs_[i].init();
            }
            inited_ = true;
            return Status::Ok;
        }

        Mae3Encoder* get(std::size_t i)
        {
            return (i < N) ? &encs_[i] : nullptr;
        }

        // Only one encoder active at a time (as per module contract)
        Status setActive(std::size_t i)
        {
            if (!inited_ || (i >= N))
            {
                return Status::InvalidArg;
            }
            for (auto& e : encs_)
            {
                (void)e.disable();
            }
            active_ = &encs_[i];
            return active_->enable();
        }

        // Observer management (thread-safe, fixed capacity)
        void attach(IEncoderObserver* obs)
        {
            portENTER_CRITICAL(&s_mux_);
            for (auto& s : observers_)
            {
                if (s == nullptr)
                {
                    s = obs;
                    break;
                }
            }
            portEXIT_CRITICAL(&s_mux_);
        }

        // Poll active encoder and notify observers (non-blocking)
        void pollAndNotify()
        {
            if (active_ == nullptr)
            {
                return;
            }

            std::double_t pos_f{0U};
            std::double_t ton_f{0U};
            std::double_t toff_f{0U};

            if (active_->tryGetPosition(pos_f, ton_f, toff_f))
            {
                // Snapshot observer pointers under lock, then notify without holding the lock.
                IEncoderObserver* local[N]{};
                portENTER_CRITICAL(&s_mux_);
                for (std::size_t i = 0; i < N; ++i)
                {
                    local[i] = observers_[i];
                }
                portEXIT_CRITICAL(&s_mux_);

                for (auto* o : local)
                {
                    if (o != nullptr)
                    {
                        o->onPositionUpdate(active_->index(), pos_f, ton_f, toff_f);
                    }
                }
            }
        }

    private:
        bool              inited_{false};
        Mae3Encoder       encs_[N];
        Mae3Encoder*      active_{nullptr};
        IEncoderObserver* observers_[N]{};  // fixed-capacity, no dynamic allocation

        static inline portMUX_TYPE s_mux_ = portMUX_INITIALIZER_UNLOCKED;
    };

}  // namespace mae3
