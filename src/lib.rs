#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_halt as _;

pub mod state_machine;

#[cfg(test)]
#[defmt_test::tests]
mod unit_tests {
    use defmt::assert;

    #[test]
    fn it_works() {
        defmt::info!("Unit Test");
        assert!(true)
    }
}
