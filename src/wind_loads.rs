use super::IO;
use crate::fem::fem_io;
use serde;
use serde::Deserialize;
use serde_pickle as pkl;
use std::error::Error;
use std::fmt;
use std::fs::File;
use std::io::BufReader;
use std::path::Path;

macro_rules! loads {
    ($($name:expr, $variant:ident),+) => {
        #[derive(Deserialize, Debug,Clone)]
        pub enum Loads {
            $(#[serde(rename = $name)]
              $variant(Vec<Vec<f64>>)),+
        }
        impl Loads {
            pub fn len(&self) -> usize {
                match self {
                    $(Loads::$variant(io) => io.len()),+
                }
            }
            pub fn io(self) -> Vec<Vec<f64>> {
                match self {
                    $(Loads::$variant(io) => io),+
                }
            }
            pub fn as_output(self) -> Outputs {
                match self {
                    $(Loads::$variant(io) => Outputs::$variant(io.into_iter())),+
                }
            }
            pub fn as_n_output(self, n: usize) -> Outputs {
                match self {
                    $(Loads::$variant(io) => Outputs::$variant(io[..n].to_owned().into_iter())),+
                }
            }
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
macro_rules! outputs {
    ($($name:expr, $variant:ident),+) => {
        pub enum Outputs {
            $($variant(std::vec::IntoIter<Vec<f64>>)),+
        }
        impl Outputs {
            pub fn len(&self) -> usize {
                match self {
                    $(Outputs::$variant(io) => io.len()),+
                }
            }
            pub fn match_io(&mut self, fem: &fem_io::Inputs) -> Option<Vec<f64>> {
                match (fem,self) {
                    $((fem_io::Inputs::$variant(_),Outputs::$variant(v)) => {
                        v.next()
                    }),+
                        _ => None
                }
            }
            pub fn next(&mut self) -> Option<Vec<f64>> {
                match self {
                    $(Outputs::$variant(v) => v.next()),+
                }
            }
        }
    };
}

outputs!(
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

#[derive(Deserialize)]
pub struct WindLoads {
    #[serde(rename = "outputs")]
    pub loads: Vec<Option<Loads>>,
    pub time: Vec<f64>,
    #[serde(skip)]
    pub n_sample: Option<usize>,
}

impl WindLoads {
    pub fn from_pickle<P>(path: P) -> Result<Self, Box<dyn Error>>
    where
        P: AsRef<Path> + fmt::Display + Copy,
    {
        let f = File::open(path)?;
        let r = BufReader::with_capacity(1_000_000_000, f);
        let v: serde_pickle::Value = serde_pickle::from_reader(r)?;
        Ok(pkl::from_value(v)?)
    }
    pub fn n_sample(self, n_sample: usize) -> Self {
        Self {
            n_sample: Some(n_sample),
            ..self
        }
    }
    fn to_io(&self, io: &IO<()>) -> Option<std::vec::IntoIter<Vec<f64>>> {
        match &self.n_sample {
            Some(n) => self
                .loads
                .iter()
                .filter_map(|x| x.as_ref().and_then(|x| io.ndata(x, *n)))
                .next(),
            None => self
                .loads
                .iter()
                .filter_map(|x| x.as_ref().and_then(|x| io.data(x)))
                .next(),
        }
    }
}
type WindItem = Option<std::vec::IntoIter<Vec<f64>>>;
/// Wind loading sources
#[derive(Default)]
pub struct WindLoading {
    pub oss_cring_6f: WindItem,
    pub oss_topend_6f: WindItem,
    pub oss_truss_6f: WindItem,
    pub oss_gir_6f: WindItem,
    pub oss_cell_lcl_6f: WindItem,
    pub oss_m1_lcl_6f: WindItem,
    pub mc_m2_lcl_6f: WindItem,
}
impl WindLoading {
    pub fn new(wind: &WindLoads, tags: &[IO<()>]) -> Result<Self, String> {
        let mut this = Self::default();
        tags.into_iter()
            .map(|t| match t {
                IO::OSSCRING6F { .. } => {
                    this.oss_cring_6f = wind.to_io(t);
                    Ok(())
                }
                IO::OSSTopEnd6F { .. } => {
                    this.oss_topend_6f = wind.to_io(t);
                    Ok(())
                }
                IO::OSSTruss6F { .. } => {
                    this.oss_truss_6f = wind.to_io(t);
                    Ok(())
                }
                IO::OSSGIR6F { .. } => {
                    this.oss_gir_6f = wind.to_io(t);
                    Ok(())
                }
                IO::OSSCellLcl6F { .. } => {
                    this.oss_cell_lcl_6f = wind.to_io(t);
                    Ok(())
                }
                IO::OSSM1Lcl6F { .. } => {
                    this.oss_m1_lcl_6f = wind.to_io(t);
                    Ok(())
                }
                IO::MCM2Lcl6F { .. } => {
                    this.mc_m2_lcl_6f = wind.to_io(t);
                    Ok(())
                }
                _ => Err(format!("Output {:?} do no belong to WindLoading", t)),
            })
            .collect::<Result<Vec<_>, _>>()
            .and(Ok(this))
    }
}
