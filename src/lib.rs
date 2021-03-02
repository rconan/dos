pub mod controllers;
pub mod io;
pub mod wind_loads;

use io::IO;
pub use wind_loads::{WindLoading, WindLoads, WindLoadsError};

use fem;
use thiserror::Error;

#[derive(Error, Debug)]
pub enum DosError {
    #[error("Windload file not found")]
    WindLoads(#[from] WindLoadsError),
}

/// DOS interface
pub trait IOTags {
    fn outputs_tags(&self) -> Vec<IO<()>>;
    fn inputs_tags(&self) -> Vec<IO<()>>;
}
pub trait DOS {
    /// Returns a `Vec` of `IO<Vec<f64>>`
    fn outputs(&mut self) -> Result<Option<Vec<IO<Vec<f64>>>>, String>;
    /// Takes in a `Vec` of `IO<Vec<f64>>`
    fn inputs(&mut self, data: Vec<IO<Vec<f64>>>) -> Result<&mut Self, String>;
    fn step(&mut self) -> Result<&mut Self, String>
    where
        Self: Sized + Iterator,
    {
        self.next()
            .and(Some(self))
            .ok_or_else(|| "DOS failed stepping the component".to_owned())
    }
}
