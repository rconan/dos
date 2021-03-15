#[derive(Clone, Debug)]
pub enum WindLoads {
    Empty,
    FileNotFound,
    PickleRead,
    Outputs,
}

#[derive(Clone, Debug)]
pub enum DOS {
    Loads(WindLoads),
    Outputs,
    Inputs,
    Step,
    IO,
}

#[derive(Clone, Debug)]
pub struct Error {
    /// Describes the kind of error that occurred.
    kind: DOS,
    /// More explanation of error that occurred.
    message: String,
}

impl Error {
    pub fn new(kind: DOS, message: &str) -> Error {
        Error {
            kind: kind,
            message: message.to_owned(),
        }
    }
}

impl From<std::io::Error> for Error {
    fn from(e: std::io::Error) -> Error {
        Error::new(DOS::IO, &format!("{}", e))
    }
}

impl From<serde_pickle::Error> for Error {
    fn from(e: serde_pickle::Error) -> Error {
        Error::new(DOS::IO, &format!("{}", e))
    }
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{:?}: {}", self.kind, self.message)
    }
}

impl std::error::Error for Error {}
