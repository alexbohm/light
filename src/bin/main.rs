#![deny(unsafe_code)]
#![deny(warnings)]
#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_halt as _;

use light as _;

#[rtic::app(device = stm32f4xx_hal::pac)]
mod app {
    use light::state_machine::LightStateMachine;
    use stm32f4xx_hal::{
        dma::{
            config::DmaConfig, traits::StreamISR, MemoryToPeripheral, Stream2, StreamsTuple,
            Transfer,
        },
        gpio::{Edge, Input, Output, PushPull, PA0, PA10, PC13},
        pac::{DMA2, SPI1},
        prelude::*,
        spi::{Mode, NoMiso, Phase, Polarity, Spi, Tx},
        timer::Event,
    };

    const ARRAY_SIZE: usize = 44;

    type TxTransfer =
        Transfer<Stream2<DMA2>, 2, Tx<SPI1>, MemoryToPeripheral, &'static mut [u8; ARRAY_SIZE]>;
    #[shared]
    struct Shared {
        state_machine: LightStateMachine,
        led: PC13<Output<PushPull>>,
        tx_transfer: TxTransfer,
    }

    #[local]
    struct Local {
        button: PA0<Input>,
        pir_sensor: PA10<Input>,
        on_deck_tx_buffer: Option<&'static mut [u8; ARRAY_SIZE]>,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut syscfg = ctx.device.SYSCFG.constrain();

        // Configure the clocks for the device. STM32CubeMX is very helpful for visualizing these values.
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc
            .cfgr
            // The Black Pill board has a 25 MHz high speed external (HSE) crystal.
            .use_hse(25.MHz())
            // Set sysclk frequency.
            .sysclk(50.MHz())
            .hclk(50.MHz())
            // Set pclk1 frequency.
            .pclk1(25.MHz())
            // Set pclk2 frequency.
            .pclk2(50.MHz())
            // Initialize the hardware.
            .freeze();

        // Initialize the GPIO blocks.
        let gpioa = ctx.device.GPIOA.split();
        let gpioc = ctx.device.GPIOC.split();

        // Initialize our timer.
        let mut timer = ctx.device.TIM2.counter_us(&clocks);
        // Enable interrupts.
        timer.listen(Event::Update);

        let state_machine = LightStateMachine::new(timer);

        // Initialize the onboard LED.
        let mut led = gpioc.pc13.into_push_pull_output();
        led.set_high();

        // Initialize the onboard button to trigger an interrupt when it is pressed.
        let mut button = gpioa.pa0.into_pull_up_input();
        button.make_interrupt_source(&mut syscfg);
        button.enable_interrupt(&mut ctx.device.EXTI);
        button.trigger_on_edge(&mut ctx.device.EXTI, Edge::Falling);

        // Initialize the PIR sensor connected to PA10 to interrupt on both rising edge.
        let mut pir_sensor = gpioa.pa10.into_floating_input();
        pir_sensor.make_interrupt_source(&mut syscfg);
        pir_sensor.enable_interrupt(&mut ctx.device.EXTI);
        pir_sensor.trigger_on_edge(&mut ctx.device.EXTI, Edge::Rising);

        // SPI Initialization.
        let spi = ctx.device.SPI1;

        let sck = gpioa.pa5.into_alternate();
        let pico = gpioa.pa7.into_alternate();

        let mode = Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnSecondTransition,
        };

        let spi1 = Spi::new(spi, (sck, NoMiso {}, pico), mode, 30.MHz(), &clocks);

        let tx = spi1.use_dma().tx();

        let streams = StreamsTuple::new(ctx.device.DMA2);
        let tx_stream = streams.2;

        let tx_buffer = cortex_m::singleton!(: [u8; ARRAY_SIZE] = [0; ARRAY_SIZE]).unwrap();
        let on_deck_tx_buffer = cortex_m::singleton!(: [u8; ARRAY_SIZE] = [0; ARRAY_SIZE]).unwrap();

        const DRIVE_CURRENT: u8 = 0b00010;

        for index in (4..ARRAY_SIZE).step_by(4) {
            tx_buffer[index + 0] = 0b11100000 | DRIVE_CURRENT;
            tx_buffer[index + 1] = state_machine.get_brightness();
            tx_buffer[index + 2] = state_machine.get_brightness();
            tx_buffer[index + 3] = state_machine.get_brightness();

            on_deck_tx_buffer[index + 0] = 0b11100000 | DRIVE_CURRENT;
        }
        for index in ARRAY_SIZE - 4..ARRAY_SIZE {
            tx_buffer[index] = 0xff;

            on_deck_tx_buffer[index] = 0xff;
        }

        let mut tx_transfer = Transfer::init_memory_to_peripheral(
            tx_stream,
            tx,
            tx_buffer,
            None,
            DmaConfig::default()
                .memory_increment(true)
                .fifo_enable(true)
                .fifo_error_interrupt(true)
                .transfer_complete_interrupt(true),
        );

        tx_transfer.start(|_tx| {});

        // Pass the hardware to the RTIC abstraction.
        (
            Shared {
                state_machine: state_machine,
                led: led,
                tx_transfer: tx_transfer,
            },
            Local {
                button: button,
                pir_sensor: pir_sensor,
                // timer: tim,
                on_deck_tx_buffer: Some(on_deck_tx_buffer),
            },
            init::Monotonics(),
        )
    }

    /// Put the CPU to sleep to conserve power until the next interrupt.
    ///
    /// This uses the Wait For Interrupt (WFI) assembly instruction to put the cpu into a low power state.
    ///
    /// Arm Documentation
    /// https://developer.arm.com/documentation/ddi0439/b/Nested-Vectored-Interrupt-Controller/NVIC-functional-description/Low-power-modes
    fn cpu_sleep_mode() {
        #[cfg(not(debug_assertions))]
        cortex_m::asm::wfi();
    }

    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            cpu_sleep_mode();
        }
    }
    /// Handle interrupts for the onboard button.
    #[task(binds = EXTI0, local = [button], shared = [state_machine, led, tx_transfer])]
    fn button_click(mut ctx: button_click::Context) {
        if ctx.local.button.check_interrupt() {
            ctx.local.button.clear_interrupt_pending_bit();

            ctx.shared.state_machine.lock(|state_machine| {
                state_machine.ir_sensor_triggered();
            });
        }
    }

    /// Handle interrupts for the PIR sensor.
    #[task(binds = EXTI15_10, local = [pir_sensor], shared = [state_machine, led])]
    fn sensor_trigger(mut ctx: sensor_trigger::Context) {
        if ctx.local.pir_sensor.check_interrupt() {
            ctx.local.pir_sensor.clear_interrupt_pending_bit();

            ctx.shared.state_machine.lock(|state_machine| {
                state_machine.ir_sensor_triggered();
            });
        }
    }

    #[task(binds = DMA2_STREAM2, local = [], shared = [tx_transfer, led])]
    fn dma2_stream2_isr(mut ctx: dma2_stream2_isr::Context) {
        if Stream2::<DMA2>::get_fifo_error_flag() {
            ctx.shared.tx_transfer.lock(|dma| {
                dma.clear_fifo_error_interrupt();
            });
        }
        if Stream2::<DMA2>::get_transfer_complete_flag() {
            ctx.shared.tx_transfer.lock(|dma| {
                dma.clear_transfer_complete_interrupt();
            });
        }
    }

    #[task(binds = TIM2, shared = [state_machine, tx_transfer], local = [on_deck_tx_buffer])]
    fn tim2_isr(mut ctx: tim2_isr::Context) {
        ctx.shared.state_machine.lock(|state_machine| {
            let event = state_machine.fade_timer.get_interrupt();
            if event | Event::Update == Event::Update {
                state_machine.fade_timer.clear_interrupt(Event::Update);

                state_machine.timer_expired();
            }
            if !event.is_empty() {
                state_machine
                    .fade_timer
                    .clear_interrupt(Event::C1 | Event::C2 | Event::C3 | Event::C4);
            }
        });

        // Grab the brightness from the state machine.
        let brightness = ctx
            .shared
            .state_machine
            .lock(|state_machine| state_machine.get_brightness());

        // Update the buffer with the new brightness.
        let buffer = ctx.local.on_deck_tx_buffer.as_mut().unwrap().as_mut();
        for index in (4..ARRAY_SIZE).step_by(4) {
            buffer[index + 1] = brightness;
            buffer[index + 2] = brightness;
            buffer[index + 3] = brightness;
        }

        // Begin transmission of the updated buffer to the LED strip and swap the old buffer into the on deck spot.
        let old_buffer = ctx.shared.tx_transfer.lock(|spi_dma| {
            let (result, _) = spi_dma
                .next_transfer(ctx.local.on_deck_tx_buffer.take().unwrap())
                .unwrap();
            result
        });
        *ctx.local.on_deck_tx_buffer = Some(old_buffer);
    }
}

// defmt-test 0.3.0 has the limitation that this `#[tests]` attribute can only be used
// once within a crate. the module can be in any file but there can only be at most
// one `#[tests]` module in this library crate
#[cfg(test)]
#[defmt_test::tests]
mod unit_tests {
    use defmt::assert;

    #[test]
    fn it_works() {
        assert!(true)
    }
}
