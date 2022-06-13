#![deny(unsafe_code)]
#![deny(warnings)]
#![no_std]
#![no_main]

use panic_halt as _;

#[rtic::app(device = stm32f4xx_hal::pac)]
mod app {
    use stm32f4xx_hal::{
        gpio::{Edge, Input, Output, PushPull, PA0, PA10, PC13},
        prelude::*,
    };

    #[shared]
    struct Shared {
        led: PC13<Output<PushPull>>,
    }

    #[local]
    struct Local {
        button: PA0<Input>,
        pir_sensor: PA10<Input>,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut syscfg = ctx.device.SYSCFG.constrain();

        // Configure the clocks for the device. STM CubeMX was used to solve for the appropriate
        // values. The sysclk is not at the maximum because if we want to use the USB or SDIO
        // module, we need the pll48clk. The pll48clk limits the sysclk because there are not enough
        // ways to multiply/divide clocks to get the appropriate values.
        let rcc = ctx.device.RCC.constrain();
        let _clocks = rcc
            .cfgr
            .use_hse(25.MHz())
            .require_pll48clk()
            .sysclk(60.MHz())
            .hclk(60.MHz())
            .pclk1(30.MHz())
            .pclk2(60.MHz())
            .freeze();

        // Initialize the GPIO blocks.
        let gpioa = ctx.device.GPIOA.split();
        let gpioc = ctx.device.GPIOC.split();

        // Initialize the onboard LED.
        let mut led = gpioc.pc13.into_push_pull_output();
        led.set_high();

        // Initialize the onboard button to trigger an interrupt when it is pressed.
        let mut button = gpioa.pa0.into_pull_up_input();
        button.make_interrupt_source(&mut syscfg);
        button.enable_interrupt(&mut ctx.device.EXTI);
        button.trigger_on_edge(&mut ctx.device.EXTI, Edge::Falling);

        // Initialize the PIR sensor connected to PA10 to interrupt on both rising and falling edges.
        let mut pir_sensor = gpioa.pa10.into_floating_input();
        pir_sensor.make_interrupt_source(&mut syscfg);
        pir_sensor.enable_interrupt(&mut ctx.device.EXTI);
        pir_sensor.trigger_on_edge(&mut ctx.device.EXTI, Edge::RisingFalling);

        // Pass the hardware to the RTIC abstraction.
        (
            Shared { led },
            Local { button, pir_sensor },
            init::Monotonics(),
        )
    }

    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        loop {}
    }
    /// Handle interrupts for the onboard button.
    #[task(binds = EXTI0, local = [button], shared = [led])]
    fn button_click(mut ctx: button_click::Context) {
        if ctx.local.button.check_interrupt() {
            ctx.local.button.clear_interrupt_pending_bit();

            // Toggle the LED atomically.
            ctx.shared.led.lock(|led| {
                led.toggle();
            })
        }
    }

    /// Handle interrupts for the PIR sensor.
    #[task(binds = EXTI15_10, local = [pir_sensor], shared = [led])]
    fn sensor_trigger(mut ctx: sensor_trigger::Context) {
        if ctx.local.pir_sensor.check_interrupt() {
            ctx.local.pir_sensor.clear_interrupt_pending_bit();

            // Toggle the LED atomically.
            ctx.shared.led.lock(|led| {
                led.toggle();
            })
        }
    }
}
