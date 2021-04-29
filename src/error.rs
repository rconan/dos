use super::{
    controllers::state_space::StateSpaceError, io::IOError, telltale::TellTaleError,
    wind_loads::WindLoadsError, DOSIOSError
};
use fem::fem::FEMError;
use std::{fmt, io};

/// The main types of DOS errors
pub enum DOSError {
    //Component(T),
    Outputs,
    Inputs(String),
    Step,
    File(io::Error),
    Pickle(serde_pickle::Error),
    IO(IOError<Vec<f64>>),
    WindLoads(WindLoadsError),
    StateSpace(StateSpaceError),
    TellTale(TellTaleError),
    FEM(FEMError),
    Other(String),
    DOSIOS(DOSIOSError),
}

impl From<std::io::Error> for DOSError {
    fn from(e: std::io::Error) -> DOSError {
        DOSError::File(e)
    }
}

impl From<serde_pickle::Error> for DOSError {
    fn from(e: serde_pickle::Error) -> DOSError {
        DOSError::Pickle(e)
    }
}

impl From<IOError<Vec<f64>>> for DOSError {
    fn from(e: IOError<Vec<f64>>) -> Self {
        Self::IO(e)
    }
}

impl From<WindLoadsError> for DOSError {
    fn from(e: WindLoadsError) -> Self {
        Self::WindLoads(e)
    }
}

impl From<StateSpaceError> for DOSError {
    fn from(e: StateSpaceError) -> Self {
        Self::StateSpace(e)
    }
}

impl From<TellTaleError> for DOSError {
    fn from(e: TellTaleError) -> Self {
        Self::TellTale(e)
    }
}

impl From<FEMError> for DOSError {
    fn from(e: FEMError) -> Self {
        Self::FEM(e)
    }
}

impl From<DOSIOSError> for DOSError {
    fn from(e: DOSIOSError) -> Self {
        Self::DOSIOS(e)
    }
}

impl From<String> for DOSError {
    fn from(e: String) -> Self {
        Self::Other(e)
    }
}
impl From<&str> for DOSError {
    fn from(e: &str) -> Self {
        Self::Other(e.to_owned())
    }
}

impl fmt::Display for DOSError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        use DOSError::*;
        match self {
            Inputs(v) => write!(f, "DOS Inputs: {}", v),
            Outputs => write!(f, "DOS Outputs failed"),
            Step => write!(f, "DOS Step failed"),
            //Component(component) => component.fmt(f),
            File(error) => error.fmt(f),
            Pickle(error) => error.fmt(f),
            IO(error) => error.fmt(f),
            WindLoads(error) => error.fmt(f),
            StateSpace(error) => error.fmt(f),
            TellTale(error) => error.fmt(f),
            FEM(error) => error.fmt(f),
            Other(error) => error.fmt(f),
            DOSIOS(error) => error.fmt(f),
        }
    }
}
impl fmt::Debug for DOSError {
		fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
			  <DOSError as std::fmt::Display>::fmt(self, f)
		}
}

impl std::error::Error for DOSError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::File(source) => Some(source),
            Self::Pickle(source) => Some(source),
            Self::IO(source) => Some(source),
            Self::WindLoads(source) => Some(source),
            Self::StateSpace(source) => Some(source),
            Self::TellTale(source) => Some(source),
            Self::FEM(source) => Some(source),
            Self::DOSIOS(source) => Some(source),
            _ => None,
        }
    }
}
