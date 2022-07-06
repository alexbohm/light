#![no_std]
#![no_main]

use light as _;

#[defmt_test::tests]
mod tests {
    use defmt::assert;

    #[test]
    fn it_works() {
        assert!(true);
    }
}
