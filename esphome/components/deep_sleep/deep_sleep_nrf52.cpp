#ifdef USE_NRF52

#include "deep_sleep_component.h"
#include "esphome/core/log.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#ifdef CONFIG_POWEROFF
#include <zephyr/sys/poweroff.h>
#endif
#include <hal/nrf_gpio.h>

namespace esphome::deep_sleep {

static const char *const TAG = "deep_sleep.nrf52";

optional<uint32_t> DeepSleepComponent::get_run_duration_() const { return this->run_duration_; }

void DeepSleepComponent::dump_config_platform_() {
  if (this->wakeup_pin_ != nullptr) {
    LOG_PIN("  Wakeup Pin: ", this->wakeup_pin_);
  }
}

bool DeepSleepComponent::prepare_to_sleep_() {
  // Mirror the BK72XX/ESP32 KEEP_AWAKE semantics: if we have no timer configured
  // and the wakeup pin is already asserted, defer sleep until the pin releases.
  if (this->wakeup_pin_ != nullptr && !this->sleep_duration_.has_value() &&
      this->wakeup_pin_mode_ == WAKEUP_PIN_MODE_KEEP_AWAKE) {
    const bool active = this->wakeup_pin_->digital_read() ^ this->wakeup_pin_->is_inverted();
    if (active) {
      if (!this->next_enter_deep_sleep_) {
        this->status_set_warning();
        ESP_LOGV(TAG, "Waiting for pin to switch state to enter deep sleep...");
      }
      this->next_enter_deep_sleep_ = true;
      return false;
    }
  }
  return true;
}

void DeepSleepComponent::deep_sleep_() {
  const bool have_pin = this->wakeup_pin_ != nullptr;
  const bool have_timer = this->sleep_duration_.has_value();

  if (have_pin) {
    // Resolve the level we want to wake on, honouring INVERT_WAKEUP.
    bool wake_high = !this->wakeup_pin_->is_inverted();
    if (this->wakeup_pin_mode_ == WAKEUP_PIN_MODE_INVERT_WAKEUP) {
      const bool active = this->wakeup_pin_->digital_read() ^ this->wakeup_pin_->is_inverted();
      if (active) {
        wake_high = !wake_high;
      }
    }
    // Arm the GPIO DETECT/SENSE latch. This is a register-level operation on
    // nRF52 and does not require a devicetree wakeup-source binding.
    nrf_gpio_cfg_sense_set(this->wakeup_pin_->get_pin(), wake_high ? NRF_GPIO_PIN_SENSE_HIGH : NRF_GPIO_PIN_SENSE_LOW);
  }

  if (have_timer) {
    // Tickless-idle sleep with the LFCLK-backed RTC running. Any duration is
    // supported; the internal 32.768 kHz crystal gives ~1.5 uA while asleep.
    // On wake (timer expiry, or the SENSE-armed GPIO if one was configured),
    // we cold-reboot so the boot flow mirrors ESP32 deep-sleep semantics.
    k_msleep(static_cast<int32_t>(*this->sleep_duration_ / 1000));
    sys_reboot(SYS_REBOOT_COLD);
  }

#ifdef CONFIG_POWEROFF
  // GPIO-only wake - enter System OFF (~0.3 uA). Waking is a full reset.
  sys_poweroff();
#endif
  // If CONFIG_POWEROFF is disabled or sys_poweroff returns unexpectedly, fall
  // back to a cold reboot so we never return to the caller.
  sys_reboot(SYS_REBOOT_COLD);
}

}  // namespace esphome::deep_sleep

#endif  // USE_NRF52
