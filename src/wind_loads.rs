//!
//! # FEM wind loads
//!
//! Provides the time series of forces and moments at different nodes of the telescope mechanical structure

use super::io::jar;
use super::IO;
use crate::fem::fem_io;
use serde;
use serde::Deserialize;
use serde_pickle as pkl;
use std::fs::File;
use std::io::BufReader;
use std::path::Path;
use thiserror::Error;

#[derive(Error, Debug)]
pub enum WindLoadsError {
    #[error("No wind loads available")]
    EmptyWindLoads,
    #[error("Windload file not found")]
    FileNotFound(#[from] std::io::Error),
    #[error("pickle reader failed")]
    PickleRead(#[from] pkl::Error),
}

//type ThisResult<T> = Result<T, Box<dyn std::error::Error>>;
type ThisResult<T> = Result<T, WindLoadsError>;
type Outputs = Option<std::vec::IntoIter<Vec<f64>>>;

macro_rules! loads {
    ($($name:expr, $variant:ident),+) => {
        /// Wind loads forces and moments
        ///
        /// A time vector containing vectors of forces and moments
        #[derive(Deserialize, Debug,Clone)]
        pub enum Loads {
            $(#[serde(rename = $name)]
              $variant(Vec<Vec<f64>>)),+
        }
        impl Loads {
            /// Returns the number of samples in the time series
            pub fn len(&self) -> usize {
                match self {
                    $(Loads::$variant(io) => io.len()),+
                }
            }
            /// Return the loads
            pub fn io(self) -> Vec<Vec<f64>> {
                match self {
                    $(Loads::$variant(io) => io),+
                }
            }
            /// Match a wind load to FEM input
            pub fn match_io(&self, fem: &fem_io::Inputs, count: usize) -> Option<&[f64]> {
                match (fem,self) {
                    $((fem_io::Inputs::$variant(_),Loads::$variant(v)) => {
                        Some(v[count].as_slice())
                    }),+
                    _ => None
                }
            }
        }
    };
}
loads!(
    "OSS_TopEnd_6F",
    OSSTopEnd6F,
    "OSS_Truss_6F",
    OSSTruss6F,
    "OSS_GIR_6F",
    OSSGIR6F,
    "OSS_CRING_6F",
    OSSCRING6F,
    "OSS_Cell_lcl_6F",
    OSSCellLcl6F,
    "OSS_M1_lcl_6F",
    OSSM1Lcl6F,
    "MC_M2_lcl_force_6F",
    MCM2Lcl6F
);

/// Wind loading sources
#[derive(Default)]
pub struct WindLoading {
    pub oss_cring_6f: Outputs,
    pub oss_topend_6f: Outputs,
    pub oss_truss_6f: Outputs,
    pub oss_gir_6f: Outputs,
    pub oss_cell_lcl_6f: Outputs,
    pub oss_m1_lcl_6f: Outputs,
    pub mc_m2_lcl_6f: Outputs,
    pub n_sample: usize,
}

/// Wind loads builder
#[derive(Deserialize)]
pub struct WindLoads {
    /// forces and moments time series
    #[serde(rename = "outputs")]
    pub loads: Vec<Option<Loads>>,
    /// time vector
    pub time: Vec<f64>,
    #[serde(skip)]
    n_sample: Option<usize>,
    #[serde(skip)]
    oss_cring_6f: Outputs,
    #[serde(skip)]
    oss_topend_6f: Outputs,
    #[serde(skip)]
    oss_truss_6f: Outputs,
    #[serde(skip)]
    oss_gir_6f: Outputs,
    #[serde(skip)]
    oss_cell_lcl_6f: Outputs,
    #[serde(skip)]
    oss_m1_lcl_6f: Outputs,
    #[serde(skip)]
    mc_m2_lcl_6f: Outputs,
}

impl WindLoads {
    /// Reads the wind loads from a pickle file
    pub fn from_pickle<P>(path: P) -> ThisResult<Self>
    where
        P: AsRef<Path> + std::fmt::Display + Copy,
    {
        let f = File::open(path)?;
        let r = BufReader::with_capacity(1_000_000_000, f);
        let v: serde_pickle::Value = serde_pickle::from_reader(r)?;
        Ok(pkl::from_value(v)?)
    }
    /// Returns the number of samples in the time series
    fn len(&self) -> ThisResult<usize> {
        self.loads
            .iter()
            .find_map(|x| x.as_ref().and_then(|x| Some(x.len())))
            .ok_or(WindLoadsError::EmptyWindLoads)
    }
    fn to_io(&self, io: &IO<()>) -> ThisResult<Outputs> {
        match &self.n_sample {
            Some(n) => self
                .loads
                .iter()
                .find_map(|x| x.as_ref().and_then(|x| io.ndata(x, *n)))
                .map_or(Err(WindLoadsError::EmptyWindLoads.into()), |x| Ok(Some(x))),
            None => self
                .loads
                .iter()
                .find_map(|x| x.as_ref().and_then(|x| io.data(x)))
                .map_or(Err(WindLoadsError::EmptyWindLoads.into()), |x| Ok(Some(x))),
        }
    }
    /// Set the number of time sample
    pub fn n_sample(self, n_sample: usize) -> ThisResult<Self> {
        assert!(n_sample > 0, "n_sample must be greater than 0");
        let n = self.len()?;
        assert!(
            n_sample <= n,
            format!(
                "n_sample cannot be greater than the number of sample ({})",
                n
            )
        );
        Ok(Self {
            n_sample: Some(if n_sample <= n { n_sample } else { n }),
            ..self
        })
    }
    /// Selects loads on the truss
    pub fn truss(self) -> ThisResult<Self> {
        Ok(Self {
            oss_truss_6f: self.to_io(&jar::OSSTruss6F::new())?,
            ..self
        })
    }
    /// Selects loads on the top-end
    pub fn topend(self) -> ThisResult<Self> {
        Ok(Self {
            oss_topend_6f: self.to_io(&jar::OSSTopEnd6F::new())?,
            ..self
        })
    }
    /// Selects loads on the C-ring
    pub fn cring(self) -> ThisResult<Self> {
        Ok(Self {
            oss_cring_6f: self.to_io(&jar::OSSCRING6F::new())?,
            ..self
        })
    }
    /// Selects loads on the GIR
    pub fn gir(self) -> ThisResult<Self> {
        Ok(Self {
            oss_gir_6f: self.to_io(&jar::OSSGIR6F::new())?,
            ..self
        })
    }
    /// Selects loads on the M1 cells
    pub fn m1_cell(self) -> ThisResult<Self> {
        Ok(Self {
            oss_cell_lcl_6f: self.to_io(&jar::OSSCellLcl6F::new())?,
            ..self
        })
    }
    /// Selects loads on the M1 segments
    pub fn m1_segments(self) -> ThisResult<Self> {
        Ok(Self {
            oss_m1_lcl_6f: self.to_io(&jar::OSSM1Lcl6F::new())?,
            ..self
        })
    }
    /// Selects loads on the M2 segments
    pub fn m2_segments(self) -> ThisResult<Self> {
        Ok(Self {
            mc_m2_lcl_6f: self.to_io(&jar::MCM2Lcl6F::new())?,
            ..self
        })
    }
    /// Selects all loads
    pub fn select_all(self) -> ThisResult<Self> {
        self.topend()?
            .m2_segments()?
            .truss()?
            .m1_segments()?
            .m1_cell()?
            .gir()?
            .cring()
    }
    /// Builds a wind loading source object
    pub fn build(self) -> ThisResult<WindLoading> {
        Ok(WindLoading {
            n_sample: self.n_sample.unwrap_or(self.len()?),
            oss_cring_6f: self.oss_cring_6f,
            oss_topend_6f: self.oss_topend_6f,
            oss_truss_6f: self.oss_truss_6f,
            oss_gir_6f: self.oss_gir_6f,
            oss_cell_lcl_6f: self.oss_cell_lcl_6f,
            oss_m1_lcl_6f: self.oss_m1_lcl_6f,
            mc_m2_lcl_6f: self.mc_m2_lcl_6f,
        })
    }
}
