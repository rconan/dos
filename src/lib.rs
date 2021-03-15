//! Dynamic optical simulation library for the Giant Magellan Telescope
//!
//! The library is used to build end-to-end simulations of the GMT.
//! It provides the following features:
//!  - the state space representation of the finite element model of the telescope,
//!  - the optical model of the telescope including ray tracing through the telescope,
//!  - the controllers of the different telescope subsystem,
//!  - the wind loads that are applied to the telescope.

pub mod controllers;
pub mod io;
pub mod telltale;
pub mod wind_loads;

use fem;
use io::IO;
#[doc(inline)]
pub use telltale::DataLogging;
use thiserror::Error;
#[doc(inline)]
pub use wind_loads::{WindLoading, WindLoads, WindLoadsError};

#[derive(Error, Debug)]
pub enum DosError {
    #[error("DOS failed stepping the component")]
    Step(),
}

/// Used to get the list of inputs or outputs
pub trait IOTags {
    /// Return the list of outputs
    fn outputs_tags(&self) -> Vec<IO<()>>;
    /// Return the list of inputs
    fn inputs_tags(&self) -> Vec<IO<()>>;
}
/// Used to glue together the different components of an end-to-end model
pub trait DOS {
    /// Computes and returns a vector outputs from a model component
    fn outputs(&mut self) -> Result<Option<Vec<IO<Vec<f64>>>>, Box<dyn std::error::Error>>;
    /// Passes a vector of input data to a model component
    fn inputs(&mut self, data: Vec<IO<Vec<f64>>>) -> Result<&mut Self, Box<dyn std::error::Error>>;
    /// Updates the state of a model component for one time step
    fn step(&mut self) -> Result<&mut Self, DosError>
    where
        Self: Sized + Iterator,
    {
        self.next().and(Some(self)).ok_or_else(|| DosError::Step())
    }
    /// Combines `inputs`, `step` and `outputs` in a single method
    ///
    /// This is equivalent to `.inputs(...)?.step()?.outputs()?`
    fn in_step_out(
        &mut self,
        data: Vec<IO<Vec<f64>>>,
    ) -> Result<Option<Vec<IO<Vec<f64>>>>, Box<dyn std::error::Error>>
    where
        Self: Sized + Iterator,
    {
        self.inputs(data)?.step()?.outputs()
    }
}
