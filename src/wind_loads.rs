//!
//! # FEM wind loads
//!
//! Provides the time series of forces and moments at different nodes of the telescope mechanical structure

use super::{
    io::{jar, Tags},
    IOTags, DOS, IO,
};
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
    #[error("Failed getting outputs")]
    Outputs(#[from] crate::io::IOError),
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
    pub loads: Vec<IO<std::vec::IntoIter<Vec<f64>>>>,
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
    /*
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
    */
    #[serde(skip)]
    tagged_loads: Vec<IO<std::vec::IntoIter<Vec<f64>>>>,
}

impl WindLoads {
    /// Reads the wind loads from a pickle file
    pub fn from_pickle<P: AsRef<Path>>(path: P) -> ThisResult<Self> {
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
    /*fn to_io(&self, io: &Tags) -> ThisResult<Outputs> {
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
    }*/
    fn tagged_load(&self, io: &Tags) -> ThisResult<Outputs> {
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
    pub fn truss(mut self) -> ThisResult<Self> {
        self.tagged_loads.push(IO::OSSTruss6F {
            data: self.tagged_load(&jar::OSSTruss6F::new())?,
        });
        Ok(self)
    }
    /// Selects loads on the top-end
    pub fn topend(mut self) -> ThisResult<Self> {
        self.tagged_loads.push(IO::OSSTopEnd6F {
            data: self.tagged_load(&jar::OSSTopEnd6F::new())?,
        });
        Ok(self)
    }
    pub fn m2_asm_topend(mut self) -> ThisResult<Self> {
        self.tagged_loads.push(IO::MCM2TE6F {
            data: self.tagged_load(&jar::OSSTopEnd6F::new())?,
        });
        Ok(self)
    }
    /// Selects loads on the C-ring
    pub fn cring(mut self) -> ThisResult<Self> {
        self.tagged_loads.push(IO::OSSCRING6F {
            data: self.tagged_load(&jar::OSSCRING6F::new())?,
        });
        Ok(self)
    }
    /// Selects loads on the GIR
    pub fn gir(mut self) -> ThisResult<Self> {
        self.tagged_loads.push(IO::OSSGIR6F {
            data: self.tagged_load(&jar::OSSGIR6F::new())?,
        });
        Ok(self)
    }
    /// Selects loads on the M1 cells
    pub fn m1_cell(mut self) -> ThisResult<Self> {
        self.tagged_loads.push(IO::OSSCellLcl6F {
            data: self.tagged_load(&jar::OSSCellLcl6F::new())?,
        });
        Ok(self)
    }
    /// Selects loads on the M1 segments
    pub fn m1_segments(mut self) -> ThisResult<Self> {
        self.tagged_loads.push(IO::OSSM1Lcl6F {
            data: self.tagged_load(&jar::OSSM1Lcl6F::new())?,
        });
        Ok(self)
    }
    /// Selects loads on the M2 segments
    pub fn m2_segments(mut self) -> ThisResult<Self> {
        self.tagged_loads.push(IO::MCM2Lcl6F {
            data: self.tagged_load(&jar::MCM2Lcl6F::new())?,
        });
        Ok(self)
    }
    pub fn m2_asm_reference_bodies(mut self) -> ThisResult<Self> {
        self.tagged_loads.push(IO::MCM2RB6F {
            data: self.tagged_load(&jar::MCM2Lcl6F::new())?,
        });
        Ok(self)
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
    /// Selects all loads in the ASM configuration
    pub fn select_all_with_asm(self) -> ThisResult<Self> {
        self.m2_asm_topend()?
            .m2_asm_reference_bodies()?
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
            loads: self.tagged_loads,
        })
    }
}
/// Wind loading interface
impl IOTags for WindLoading {
    fn outputs_tags(&self) -> Vec<Tags> {
        self.loads.iter().map(|x| x.into()).collect()
    }
    fn inputs_tags(&self) -> Vec<Tags> {
        unimplemented!("WindLoading takes no inputs")
    }
}
impl DOS for WindLoading {
    fn inputs(&mut self, _: Vec<IO<Vec<f64>>>) -> Result<&mut Self, Box<dyn std::error::Error>> {
        unimplemented!()
    }
    fn outputs(&mut self) -> Result<Option<Vec<IO<Vec<f64>>>>, Box<dyn std::error::Error>> {
        self.loads
            .iter_mut()
            .map(|x| -> Result<Option<IO<Vec<f64>>>, Box<dyn std::error::Error>> { x.into() })
            .collect()
        /*Ok(Some(vec![
            IO::OSSTopEnd6F {
                data: Some(
                    self.oss_topend_6f
                        .as_mut()
                        .ok_or_else(|| "OSSTopEnd6F not available")?
                        .next()
                        .ok_or_else(|| "Empty")?,
                ),
            },
            IO::MCM2Lcl6F {
                data: Some(
                    self.mc_m2_lcl_6f
                        .as_mut()
                        .ok_or_else(|| "MCM2Lcl6F not available")?
                        .next()
                        .ok_or_else(|| "Empty")?,
                ),
            },
            IO::OSSTruss6F {
                data: Some(
                    self.oss_truss_6f
                        .as_mut()
                        .ok_or_else(|| "OSSTruss6F not available")?
                        .next()
                        .ok_or_else(|| "Empty")?,
                ),
            },
            IO::OSSM1Lcl6F {
                data: Some(
                    self.oss_m1_lcl_6f
                        .as_mut()
                        .ok_or_else(|| "OSSM1Lcl6F not available")?
                        .next()
                        .ok_or_else(|| "Empty")?,
                ),
            },
            IO::OSSCellLcl6F {
                data: Some(
                    self.oss_cell_lcl_6f
                        .as_mut()
                        .ok_or_else(|| "OSSCellLcl6F not available")?
                        .next()
                        .ok_or_else(|| "Empty")?,
                ),
            },
            IO::OSSGIR6F {
                data: Some(
                    self.oss_gir_6f
                        .as_mut()
                        .ok_or_else(|| "OSSGIR6F not available")?
                        .next()
                        .ok_or_else(|| "Empty")?,
                ),
            },
            IO::OSSCRING6F {
                data: Some(
                    self.oss_cring_6f
                        .as_mut()
                        .ok_or_else(|| "OSSCRING6F not available")?
                        .next()
                        .ok_or_else(|| "Empty")?,
                ),
            },
        ]))*/
    }
}
