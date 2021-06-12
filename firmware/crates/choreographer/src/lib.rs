#![cfg_attr(not(test), no_std)]

pub mod behaviors;
pub mod engine;

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
