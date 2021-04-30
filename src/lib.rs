//! Dynamic optical simulation library for the Giant Magellan Telescope
//!
//! The library is used to build end-to-end simulations of the GMT.
//! It provides the following features:
//!  - the state space representation of the finite element model of the telescope,
//!  - the optical model of the telescope including ray tracing through the telescope,
//!  - the controllers of the different telescope subsystem,
//!  - the wind loads that are applied to the telescope.
//!
//! An end-to-end simulation is divided into components, each represented by a structure:
//!  - [`DiscreteModalsolver`](crate::controllers::state_space::DiscreteStateSpace) for the finite element model of the telescope ([example](crate::controllers::state_space)),
//!  - [`WindLoading`] for the wind loads,
//!  - `Controller` for each subsystem controller.
//!
//! Each component structure contains a [`Vec`] of either inputs, outputs or both that corresponds to some variant of the [`IO`] enum type.
//! Each component structure must implement the [`Iterator`] and the [`DOS`] traits.
//! The [`next`](core::iter::Iterator::next) method of the [`Iterator`] trait is used to update the state of the component at each time step.
//! The [`inputs`](crate::DOS::inputs) method of the [`DOS`] trait passes inputs data to the components whereas the [`outputs`](crate::DOS::outputs) method returns the component outputs.

pub mod controllers;
pub mod error;
pub mod telltale;
pub mod wind_loads;

pub use error::DOSError;
use fem;
#[doc(inline)]
pub use telltale::DataLogging;
#[doc(inline)]
pub use wind_loads::{WindLoading, WindLoads};

pub mod match_io;
pub mod io {
    pub use super::match_io::{MatchFEM, MatchWindLoads};
    pub use dosio::io::*;
}
pub use dosio::{DOSIOSError, DOS};
pub use io::IO;

/// Used to get the list of inputs or outputs
pub trait IOTags {
    /// Return the list of outputs
    fn outputs_tags(&self) -> Vec<IO<()>>;
    /// Return the list of inputs
    fn inputs_tags(&self) -> Vec<IO<()>>;
}
