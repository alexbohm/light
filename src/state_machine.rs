use defmt::Format;
use stm32f4xx_hal::{pac::TIM2, prelude::*, timer::Counter};

#[derive(Debug, Format, Clone, Copy)]
pub enum LightState {
    On,
    FadeOff,
    Off,
    FadeOn,
}

const FADE_OFF_DELAY: u32 = 10 * 1000;
const FADE_ON_DELAY: u32 = 1 * 1000;
const TIMEOUT_DELAY: u32 = 5 * 60 * 1000;

pub struct LightStateMachine {
    pub fade_timer: Counter<TIM2, 1000000>,
    state: LightState,
    brightness: u8,
}

impl LightStateMachine {
    pub fn new(fade_timer: Counter<TIM2, 1000000>) -> Self {
        let mut m = LightStateMachine {
            fade_timer: fade_timer,
            state: LightState::On,
            brightness: u8::MAX,
        };

        m.transition(LightState::On);

        m
    }
    pub fn get_state(&self) -> &LightState {
        &self.state
    }

    pub fn get_brightness(&self) -> u8 {
        self.brightness
    }

    fn transition(&mut self, new_state: LightState) {
        match new_state {
            LightState::FadeOff => {
                const D: u32 = FADE_OFF_DELAY * 1000 / u8::MAX as u32;
                self.fade_timer.start(D.micros()).unwrap();
            }
            LightState::FadeOn => {
                const D: u32 = FADE_ON_DELAY * 1000 / u8::MAX as u32;
                self.fade_timer.start(D.micros()).unwrap();
            }
            LightState::On => {
                self.fade_timer.start(TIMEOUT_DELAY.millis()).unwrap();
            }
            LightState::Off => {
                self.fade_timer.cancel().unwrap();
            }
        }
        defmt::info!("Transitioning to state: {}", new_state);
        self.state = new_state;
    }

    pub fn timer_expired(&mut self) {
        match &self.state {
            LightState::FadeOff => {
                self.brightness = self.brightness.saturating_sub(1);

                if self.brightness == 0 {
                    self.transition(LightState::Off);
                }
            }
            LightState::FadeOn => {
                self.brightness = self.brightness.saturating_add(1);

                if self.brightness == 255 {
                    self.transition(LightState::On);
                }
            }
            LightState::On => {
                self.transition(LightState::FadeOff);
            }
            state => {
                panic!(
                    "Fade timer expired when not in a state that should have the timer running. state: {:?}", state
                );
            }
        }
    }

    pub fn ir_sensor_triggered(&mut self) {
        match &self.state {
            LightState::On => {
                self.transition(LightState::On);
            }
            LightState::FadeOn => {
                // Do Nothing.
            }
            LightState::FadeOff => {
                self.transition(LightState::FadeOn);
            }
            LightState::Off => {
                self.transition(LightState::FadeOn);
            }
        }
    }
}
