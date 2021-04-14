use std::{fmt,io};
use super::io::IOError;

/// The main types of DOS errors
#[derive(Debug)]
pub enum DOSError {
    //Component(T),
    Outputs,
    Inputs(String),
    Step,
    File(io::Error),
    Pickle(serde_pickle::Error),
    IO(IOError<Vec<f64>>)
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

impl fmt::Display for DOSError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        use DOSError::*;
        match self {
            Inputs(v) => write!(f, "DOS Inputs: {}",v),
            Outputs => write!(f, "DOS Outputs failed"),
            Step => write!(f, "DOS Step failed"),
            //Component(component) => component.fmt(f),
            File(error) => error.fmt(f),
            Pickle(error) => error.fmt(f),
            IO(error) => error.fmt(f),
        }
    }
}

impl std::error::Error for DOSError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::File(source) => Some(source),
            Self::Pickle(source) => Some(source),
            Self::IO(source) => Some(source),
            _ => None,
        }
    }
}
