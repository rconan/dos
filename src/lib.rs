use core::fmt::Debug;
use fem;
use gmt_controllers as ctrlr;
use nalgebra as na;
use serde::Serialize;

/// DOS interface
pub trait DOS<T> {
    /// Returns a `Vec` of `IO<Vec<f64>>`
    fn outputs(&mut self, tags: &[IO<T>]) -> Result<Vec<IO<Vec<f64>>>, String>;
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

macro_rules! build_io {
    ($($variant:ident),+) => {
        /// DOS inputs/ouputs
        #[derive(Debug,Clone,Serialize)]
        pub enum IO<T> {
            $($variant{data: Option<T>}),+
        }
        impl IO<usize> {
            /// Assign `n` to `IO` `data`
            fn assign(&mut self, n: usize) {
                match self {
                    $(IO::$variant{ data: values} => {*values=Some(n);}),+
                }
            }
        }
        impl<T> From<IO<T>> for Option<T> {
            /// Converts a `IO<T>` into an `Option<T>`
            fn from(io: IO<T>) -> Self {
                match io {
                    $(IO::$variant{ data: values} => values),+
                }
            }
        }
        impl<T: Debug> From<IO<T>> for Result<T,String> {
            /// Converts a `IO<T>` into an `Option<T>`
            fn from(io: IO<T>) -> Self {
                match io {
                    $(IO::$variant{ data: values} => values.ok_or_else(|| "Data missing".to_owned())),+
                }
            }
        }
        impl<T: Clone> From<&IO<T>> for Option<T> {
            /// Converts a `&IO<T>` into an `Option<T>`
            fn from(io: &IO<T>) -> Self {
                match io {
                    $(IO::$variant{ data: values} => values.as_ref().cloned()),+
                }
            }
        }
        impl From<(&IO<usize>,Vec<f64>)> for IO<Vec<f64>> {
            /// Converts a `(&IO<usize>,Vec<f64>)` into an `IO<Vec<f64>>`
            fn from((io,v): (&IO<usize>,Vec<f64>)) -> Self {
                match io {
                    $(IO::$variant{ data: _} => IO::$variant{ data: Some(v)}),+
                }
            }
        }
    };
}
macro_rules! io_match_fem {
    (inputs: ($($inputs_variant:ident),+), outputs: ($($outputs_variant:ident),+)) => {
        impl<T: Debug> IO<T> {
            /// Matches a FEM input to a DOS `IO` returning the FEM input value
            fn match_fem_inputs(&self, fem_inputs: &fem::fem_io::Inputs) -> Option<Vec<fem::IO>> {
                match (self,fem_inputs) {
                    $((IO::$inputs_variant{data: _}, fem::fem_io::Inputs::$inputs_variant(v)) => {
                        Some(v.clone())},)+
                    (_, _) => None,
                }
            }
            /// Matches a FEM output to a DOS `IO` returning the FEM output value
            fn match_fem_outputs(&self, fem_outputs: &fem::fem_io::Outputs) -> Option<Vec<fem::IO>> {
                match (self,fem_outputs) {
                    $((IO::$outputs_variant{data: _}, fem::fem_io::Outputs::$outputs_variant(v)) => Some(v.clone()),)+
                        (_, _) => None,
                }
            }
        }
    };
}
macro_rules! io_match_wind_loads {
    ($($variant:ident),+) => {
        impl<T> IO<T> {
            /// Matches a wind loads to a DOS `IO` returning the wind load value as an iterator over the first `n` elements
            fn data(&self, wind_loads: &fem::wind_loads::Loads) -> Option<std::vec::IntoIter<Vec<f64>>> {
                match (self,wind_loads) {
                    $((IO::$variant{data: _}, fem::wind_loads::Loads::$variant(v)) => Some(v.clone().into_iter()),)+
                        (_, _) => None,
                }
            }
            /// Matches a wind loads to a DOS `IO` returning the wind load value as an iterator
            fn ndata(&self, wind_loads: &fem::wind_loads::Loads, n: usize) -> Option<std::vec::IntoIter<Vec<f64>>> {
                match (self,wind_loads) {
                    $((IO::$variant{data: _}, fem::wind_loads::Loads::$variant(v)) => Some(v[..n].to_owned().into_iter()),)+
                        (_, _) => None,
                }
            }
        }
    };
}

build_io!(
    OSSTopEnd6F,
    OSSTruss6F,
    OSSGIR6F,
    OSSTopEnd6D,
    OSSTruss6D,
    OSSGIR6D,
    OSSCRING6F,
    OSSCellLcl6F,
    OSSM1Lcl6F,
    MCM2Lcl6F,
    OSSM1Lcl,
    MCM2Lcl6D,
    CMD,
    OSSAzDriveF,
    OSSElDriveF,
    OSSGIRDriveF,
    OSSAzDriveD,
    OSSElDriveD,
    OSSGIRDriveD
);
io_match_fem!(
    inputs:
        (
            OSSTopEnd6F,
            OSSTruss6F,
            OSSGIR6F,
            OSSCRING6F,
            OSSCellLcl6F,
            OSSM1Lcl6F,
            MCM2Lcl6F,
            OSSAzDriveF,
            OSSElDriveF,
            OSSGIRDriveF
        ),
    outputs: (OSSAzDriveD, OSSElDriveD, OSSGIRDriveD, OSSM1Lcl, MCM2Lcl6D)
);
io_match_wind_loads!(
    OSSTopEnd6F,
    OSSTruss6F,
    OSSGIR6F,
    OSSCRING6F,
    OSSCellLcl6F,
    OSSM1Lcl6F,
    MCM2Lcl6F
);

/// FEM to DOS interface
trait DOSFEM {
    fn io2modes(&self, dos_inputs: &[IO<()>]) -> Result<Vec<f64>, String>;
    fn modes2io(&self, dos_outputs: &mut [IO<usize>]) -> Result<Vec<Vec<f64>>, String>;
    //    fn newish(fem: &mut FEM, sampling_rate: f64) -> Self;
}
impl DOSFEM for fem::FEM {
    fn io2modes(&self, dos_inputs: &[IO<()>]) -> Result<Vec<f64>, String> {
        use fem::IO;
        let indices: Vec<u32> = dos_inputs
            .iter()
            .map(|x| {
                self.inputs
                    .iter()
                    .find_map(|y| y.as_ref().and_then(|y| x.match_fem_inputs(y)))
                    .ok_or(format!("No FEM inputs match DOS {:?}", x))
            })
            .collect::<Result<Vec<Vec<IO>>, String>>()?
            .iter()
            .flat_map(|v| {
                v.iter().filter_map(|x| match x {
                    IO::On(io) => Some(io.indices.clone()),
                    IO::Off(_) => None,
                })
            })
            .flatten()
            .collect();
        let n = self.inputs_to_modal_forces.len() / self.n_modes();
        Ok(self
            .inputs_to_modal_forces
            .chunks(n)
            .flat_map(|x| {
                indices
                    .iter()
                    .map(|i| x[*i as usize - 1])
                    .collect::<Vec<f64>>()
            })
            .collect())
    }
    fn modes2io(&self, dos_outputs: &mut [IO<usize>]) -> Result<Vec<Vec<f64>>, String> {
        use fem::IO;
        let n = self.n_modes();
        let q: Vec<_> = self.modal_disp_to_outputs.chunks(n).collect();
        Ok(dos_outputs
            .iter()
            .map(|x| {
                self.outputs
                    .iter()
                    .find_map(|y| y.as_ref().and_then(|y| x.match_fem_outputs(y)))
                    .ok_or(format!("No FEM outputs match DOS {:?}", x))
            })
            .collect::<Result<Vec<Vec<IO>>, String>>()?
            .into_iter()
            .map(|v| {
                v.into_iter()
                    .filter_map(|x| match x {
                        IO::On(io) => Some(io.indices),
                        IO::Off(_) => None,
                    })
                    .flatten()
                    .collect::<Vec<u32>>()
            })
            .map(|i| {
                i.iter()
                    .flat_map(|i| q[*i as usize - 1].to_owned())
                    .collect::<Vec<f64>>()
            })
            .collect())
    }
}
/// FEM `DiscreteModalSolver` to `DOS` interface
pub trait DOSDiscreteModalSolver {
    fn new(
        sampling_rate: f64,
        fem: &mut fem::FEM,
        dos_inputs: &[IO<()>],
        dos_outputs: &mut [IO<usize>],
    ) -> Result<Self, String>
    where
        Self: Sized;
}
impl DOSDiscreteModalSolver for fem::DiscreteModalSolver {
    /// Creates a `DiscreteModalSolver` based on the `sampling_rate` of a `fem` extracting from the FEM the inputs and outputs that matches `dos_inputs` and `dos_outputs`, the outputs size are written in `dos_outputs`
    fn new(
        sampling_rate: f64,
        fem: &mut fem::FEM,
        dos_inputs: &[IO<()>],
        dos_outputs: &mut [IO<usize>],
    ) -> Result<Self, String> {
        let tau = 1. / sampling_rate;
        let forces_2_modes =
            na::DMatrix::from_row_slice(fem.n_modes(), fem.n_inputs(), &fem.io2modes(dos_inputs)?);
        println!("forces 2 modes: {:?}", forces_2_modes.shape());
        let fem_modes2io = fem.modes2io(dos_outputs)?;
        fem_modes2io.iter().zip(dos_outputs).for_each(|(f, o)| {
            let n = f.len() / fem.n_modes();
            o.assign(n);
        });
        let modes_2_nodes = na::DMatrix::from_row_slice(
            fem.n_outputs(),
            fem.n_modes(),
            &fem_modes2io.into_iter().flatten().collect::<Vec<f64>>(),
        );
        println!("modes 2 nodes: {:?}", modes_2_nodes.shape());
        let w = fem.eigen_frequencies_to_radians();
        let zeta = &fem.proportional_damping_vec;
        let state_space: Vec<fem::Exponential> = (0..fem.n_modes())
            .map(|k| {
                let b = forces_2_modes.row(k);
                let c = modes_2_nodes.column(k);
                fem::Exponential::from_second_order(
                    tau,
                    w[k],
                    zeta[k],
                    b.clone_owned().as_slice().to_vec(),
                    c.as_slice().to_vec(),
                )
            })
            .collect();
        Ok(Self {
            u: vec![0f64; forces_2_modes.ncols()],
            y: vec![0f64; modes_2_nodes.nrows()],
            state_space,
        })
    }
}
impl DOS<usize> for fem::DiscreteModalSolver {
    fn inputs(&mut self, data: Vec<IO<Vec<f64>>>) -> Result<&mut Self, String> {
        self.u = data
            .into_iter()
            .map(|x| Result::<Vec<f64>, String>::from(x))
            .collect::<Result<Vec<Vec<f64>>, String>>()?
            .into_iter()
            .flatten()
            .collect();
        Ok(self)
    }
    fn outputs(&mut self, tags: &[IO<usize>]) -> Result<Vec<IO<Vec<f64>>>, String> {
        let mut pos = 0;
        tags.iter()
            .map(|t| {
                let n = Option::<usize>::from(t)
                    .ok_or_else(|| "No given size passed to DiscreModalSolver outputs")?;
                let io = IO::<Vec<f64>>::from((t, self.y[pos..pos + n].to_vec()));
                pos += n;
                Ok(io)
            })
            .collect()
    }
}
/// `WindLoads` to `DOS` Interface
trait DOSWindLoads {
    fn to_io(&self, io: &IO<()>) -> Option<std::vec::IntoIter<Vec<f64>>>;
}
impl DOSWindLoads for fem::wind_loads::WindLoads {
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
    pub fn new(wind: &fem::wind_loads::WindLoads, tags: &[IO<()>]) -> Result<Self, String> {
        let mut this = Self::default();
        tags.into_iter()
            .map(|t| match t {
                IO::OSSCRING6F { data: _ } => {
                    this.oss_cring_6f = wind.to_io(t);
                    Ok(())
                }
                IO::OSSTopEnd6F { data: _ } => {
                    this.oss_topend_6f = wind.to_io(t);
                    Ok(())
                }
                IO::OSSTruss6F { data: _ } => {
                    this.oss_truss_6f = wind.to_io(t);
                    Ok(())
                }
                IO::OSSGIR6F { data: _ } => {
                    this.oss_gir_6f = wind.to_io(t);
                    Ok(())
                }
                IO::OSSCellLcl6F { data: _ } => {
                    this.oss_cell_lcl_6f = wind.to_io(t);
                    Ok(())
                }
                IO::OSSM1Lcl6F { data: _ } => {
                    this.oss_m1_lcl_6f = wind.to_io(t);
                    Ok(())
                }
                IO::MCM2Lcl6F { data: _ } => {
                    this.mc_m2_lcl_6f = wind.to_io(t);
                    Ok(())
                }
                _ => Err(format!("Output {:?} do no belong to WindLoading", t)),
            })
            .collect::<Result<Vec<_>, _>>()
            .and(Ok(this))
    }
}
impl DOS<()> for WindLoading {
    fn inputs(&mut self, _: Vec<IO<Vec<f64>>>) -> Result<&mut Self, String> {
        unimplemented!()
    }
    fn outputs(&mut self, tags: &[IO<()>]) -> Result<Vec<IO<Vec<f64>>>, String> {
        tags.into_iter()
            .map(|t| match t {
                IO::OSSCRING6F { data: _ } => Ok(IO::OSSCRING6F {
                    data: Some(
                        self.oss_cring_6f
                            .as_mut()
                            .ok_or_else(|| "OSSCRING6F not available")?
                            .next()
                            .ok_or_else(|| "Empty")?,
                    ),
                }),
                IO::OSSTopEnd6F { data: _ } => Ok(IO::OSSTopEnd6F {
                    data: Some(
                        self.oss_topend_6f
                            .as_mut()
                            .ok_or_else(|| "OSSTopEnd6F not available")?
                            .next()
                            .ok_or_else(|| "Empty")?,
                    ),
                }),
                IO::OSSTruss6F { data: _ } => Ok(IO::OSSTruss6F {
                    data: Some(
                        self.oss_truss_6f
                            .as_mut()
                            .ok_or_else(|| "OSSTruss6F not available")?
                            .next()
                            .ok_or_else(|| "Empty")?,
                    ),
                }),
                IO::OSSGIR6F { data: _ } => Ok(IO::OSSGIR6F {
                    data: Some(
                        self.oss_gir_6f
                            .as_mut()
                            .ok_or_else(|| "OSSGIR6F not available")?
                            .next()
                            .ok_or_else(|| "Empty")?,
                    ),
                }),
                IO::OSSCellLcl6F { data: _ } => Ok(IO::OSSCellLcl6F {
                    data: Some(
                        self.oss_cell_lcl_6f
                            .as_mut()
                            .ok_or_else(|| "OSSCellLcl6F not available")?
                            .next()
                            .ok_or_else(|| "Empty")?,
                    ),
                }),
                IO::OSSM1Lcl6F { data: _ } => Ok(IO::OSSM1Lcl6F {
                    data: Some(
                        self.oss_m1_lcl_6f
                            .as_mut()
                            .ok_or_else(|| "OSSM1Lcl6F not available")?
                            .next()
                            .ok_or_else(|| "Empty")?,
                    ),
                }),
                IO::MCM2Lcl6F { data: _ } => Ok(IO::MCM2Lcl6F {
                    data: Some(
                        self.mc_m2_lcl_6f
                            .as_mut()
                            .ok_or_else(|| "MCM2Lcl6F not available")?
                            .next()
                            .ok_or_else(|| "Empty")?,
                    ),
                }),
                _ => Err(format!("Output {:?} do no belong to WindLoading", t)),
            })
            .collect()
    }
}
// Mount
impl<'a> DOS<usize> for ctrlr::mount::drives::Controller<'a> {
    fn inputs(&mut self, data: Vec<IO<Vec<f64>>>) -> Result<&mut Self, String> {
        data.into_iter()
            .find_map(|io| match io {
                IO::CMD { data: Some(values) } => {
                    self.cmd[0] = values[0];
                    self.cmd[1] = values[1];
                    self.cmd[2] = values[2];
                    Some(())
                }
                _ => None,
            })
            .and(Some(self))
            .ok_or_else(|| "Missing CMD DOS IO for mount drives controller".to_owned())
    }
    fn outputs(&mut self, tags: &[IO<usize>]) -> Result<Vec<IO<Vec<f64>>>, String> {
        let mut pos = 0;
        let mut check = 3;
        tags.iter()
            .fold(Vec::<Result<IO<Vec<f64>>, String>>::new(), |mut a, t| {
                match t {
                    IO::OSSAzDriveF { data: Some(n) } => {
                        a.push(Ok(IO::OSSAzDriveF {
                            data: Some(
                                Vec::<f64>::from(&self.oss_az_drive_f)[pos..pos + n].to_vec(),
                            ),
                        }));
                        pos += n;
                        check -= 1;
                    }
                    IO::OSSElDriveF { data: Some(n) } => {
                        a.push(Ok(IO::OSSElDriveF {
                            data: Some(
                                Vec::<f64>::from(&self.oss_el_drive_f)[pos..pos + n].to_vec(),
                            ),
                        }));
                        pos += n;
                        check -= 1;
                    }
                    IO::OSSGIRDriveF { data: Some(n) } => {
                        a.push(Ok(IO::OSSGIRDriveF {
                            data: Some(
                                Vec::<f64>::from(&self.oss_gir_drive_f)[pos..pos + n].to_vec(),
                            ),
                        }));
                        pos += n;
                        check -= 1;
                    }
                    _ => (),
                }
                if a.len()==3 {return a}
                a
            })
            .into_iter()
            .collect::<Result<Vec<_>, _>>()
            .and_then(|v| {
                if check == 0 {
                    Ok(v)
                } else {
                    Err("Missing DOS IO in mount drives controller".to_owned())
                }
            })
        /*
        tags.iter()
            .filter_map(|t| match t {
                IO::OSSAzDriveF { data: Some(n) } => {
                    let io = IO::OSSAzDriveF {
                        data: Some(Vec::<f64>::from(&self.oss_az_drive_f)[pos..pos + n].to_vec()),
                    };
                    pos += n;
                   Some(io)
                }
                IO::OSSElDriveF { data: Some(n) } => {
                    let io = IO::OSSElDriveF {
                        data: Some(Vec::<f64>::from(&self.oss_el_drive_f)[pos..pos + n].to_vec()),
                    };
                    pos += n;
                    Some(io)
                }
                IO::OSSGIRDriveF { data: Some(n) } => {
                    let io = IO::OSSGIRDriveF {
                        data: Some(Vec::<f64>::from(&self.oss_gir_drive_f)[pos..pos + n].to_vec()),
                    };
                    pos += n;
                    Some(io)
                }
                _ => None//Err("Unexpected DOS IO in mount drives controller".to_owned()),
            })
            .collect::Vec<IO<Vec<f64>>>()*/
    }
}
impl<'a> DOS<usize> for ctrlr::mount::controller::Controller<'a> {
    fn inputs(&mut self, data: Vec<IO<Vec<f64>>>) -> Result<&mut Self, String> {
        if data.into_iter().fold(3, |mut a, io| {
            match io {
                IO::OSSAzDriveD { data: Some(values) } => {
                    for (k, v) in values.into_iter().enumerate() {
                        self.oss_az_drive[k] = v;
                    }
                    a -= 1;
                }
                IO::OSSElDriveD { data: Some(values) } => {
                    for (k, v) in values.into_iter().enumerate() {
                        self.oss_el_drive[k] = v;
                    }
                    a -= 1;
                }
                IO::OSSGIRDriveD { data: Some(values) } => {
                    for (k, v) in values.into_iter().enumerate() {
                        self.oss_gir_drive[k] = v;
                    }
                    a -= 1;
                }
                _ => (),
            }
            if a==0 {return a}
            a
        }) == 0
        {
            Ok(self)
        } else {
            Err("Either mount controller controller OSSAzDriveD, OSSElDriveD or OSSGIRDriveD not found".to_owned())
        }
        /*
        data.into_iter()
            .map(|io| match io {
                IO::OSSAzDriveD { data: Some(values) } => {
                    for (k,v) in values.into_iter().enumerate() {
                        self.oss_az_drive[k] = v;
                    }
                    Ok(())
                }
                IO::OSSElDriveD { data: Some(values) } => {
                    for (k,v) in values.into_iter().enumerate() {
                        self.oss_el_drive[k] = v;
                    }
                    Ok(())
                }
                IO::OSSGIRDriveD { data: Some(values) } => {
                    for (k,v) in values.into_iter().enumerate() {
                        self.oss_gir_drive[k] = v;
                    }
                    Ok(())
                }
                _ => Err("Either mount controller controller OSSAzDriveD, OSSElDriveD or OSSGIRDriveD not found".to_owned()),
            })
            .collect::<Result<(), String>>()
            .and(Ok(self))*/
    }
    fn outputs(&mut self, tags: &[IO<usize>]) -> Result<Vec<IO<Vec<f64>>>, String> {
        tags.iter()
            .find_map(|t| match t {
                IO::CMD { data: _ } => Some(IO::CMD {
                    data: Some(Vec::<f64>::from(&self.cmd)),
                }),
                _ => None,
            })
            .and_then(|x| Some(vec![x]))
            .ok_or_else(|| "Unexpected DOS IO in mount controller controller".to_owned())
    }
}
//Err(),
