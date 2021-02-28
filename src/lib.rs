pub mod io;
pub mod wind_loads;

use io::IO;
pub use wind_loads::{WindLoads,WindLoading};

use fem;
use gmt_controllers as ctrlr;
use nalgebra as na;

/// DOS interface
pub trait DOS<T, U> {
    /// Returns a `Vec` of `IO<Vec<f64>>`
    fn outputs(&mut self, tags: &[IO<T>]) -> Result<Option<Vec<IO<U>>>, String>;
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
impl DOS<usize, Vec<f64>> for fem::DiscreteModalSolver {
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
    fn outputs(&mut self, tags: &[IO<usize>]) -> Result<Option<Vec<IO<Vec<f64>>>>, String> {
        let mut pos = 0;
        tags.iter()
            .map(|t| {
                let n = Option::<usize>::from(t)
                    .ok_or_else(|| "No given size passed to DiscreModalSolver outputs")?;
                let io = IO::<Vec<f64>>::from((t, self.y[pos..pos + n].to_vec()));
                pos += n;
                Ok(Some(io))
            })
            .collect()
    }
}
/// Wind loading interface
impl DOS<(), Vec<f64>> for WindLoading {
    fn inputs(&mut self, _: Vec<IO<Vec<f64>>>) -> Result<&mut Self, String> {
        unimplemented!()
    }
    fn outputs(&mut self, tags: &[IO<()>]) -> Result<Option<Vec<IO<Vec<f64>>>>, String> {
        tags.into_iter()
            .map(|t| match t {
                IO::OSSCRING6F { .. } => Ok(Some(IO::OSSCRING6F {
                    data: Some(
                        self.oss_cring_6f
                            .as_mut()
                            .ok_or_else(|| "OSSCRING6F not available")?
                            .next()
                            .ok_or_else(|| "Empty")?,
                    ),
                })),
                IO::OSSTopEnd6F { .. } => Ok(Some(IO::OSSTopEnd6F {
                    data: Some(
                        self.oss_topend_6f
                            .as_mut()
                            .ok_or_else(|| "OSSTopEnd6F not available")?
                            .next()
                            .ok_or_else(|| "Empty")?,
                    ),
                })),
                IO::MCM2TE6F { .. } => Ok(Some(IO::MCM2TE6F {
                    data: Some(
                        self.oss_topend_6f
                            .as_mut()
                            .ok_or_else(|| "OSSTopEnd6F not available")?
                            .next()
                            .ok_or_else(|| "Empty")?,
                    ),
                })),
                IO::OSSTruss6F { .. } => Ok(Some(IO::OSSTruss6F {
                    data: Some(
                        self.oss_truss_6f
                            .as_mut()
                            .ok_or_else(|| "OSSTruss6F not available")?
                            .next()
                            .ok_or_else(|| "Empty")?,
                    ),
                })),
                IO::OSSGIR6F { .. } => Ok(Some(IO::OSSGIR6F {
                    data: Some(
                        self.oss_gir_6f
                            .as_mut()
                            .ok_or_else(|| "OSSGIR6F not available")?
                            .next()
                            .ok_or_else(|| "Empty")?,
                    ),
                })),
                IO::OSSCellLcl6F { .. } => Ok(Some(IO::OSSCellLcl6F {
                    data: Some(
                        self.oss_cell_lcl_6f
                            .as_mut()
                            .ok_or_else(|| "OSSCellLcl6F not available")?
                            .next()
                            .ok_or_else(|| "Empty")?,
                    ),
                })),
                IO::OSSM1Lcl6F { .. } => Ok(Some(IO::OSSM1Lcl6F {
                    data: Some(
                        self.oss_m1_lcl_6f
                            .as_mut()
                            .ok_or_else(|| "OSSM1Lcl6F not available")?
                            .next()
                            .ok_or_else(|| "Empty")?,
                    ),
                })),
                IO::MCM2Lcl6F { .. } => Ok(Some(IO::MCM2Lcl6F {
                    data: Some(
                        self.mc_m2_lcl_6f
                            .as_mut()
                            .ok_or_else(|| "MCM2Lcl6F not available")?
                            .next()
                            .ok_or_else(|| "Empty")?,
                    ),
                })),
                _ => Err(format!("Output {:?} do no belong to WindLoading", t)),
            })
            .collect()
    }
}
// Mount
impl<'a> DOS<(), Vec<f64>> for ctrlr::mount::drives::Controller<'a> {
    fn inputs(&mut self, data: Vec<IO<Vec<f64>>>) -> Result<&mut Self, String> {
        if data.into_iter().fold(4, |mut a, io| {
            match io {
                IO::CMD { data: Some(values) } => {
                    for (k, v) in values.into_iter().enumerate() {
                        self.cmd[k] = v;
                    }
                    a -= 1;
                }
                IO::OSSAzDriveD { data: Some(values) } => {
                    for (k, v) in values.into_iter().enumerate() {
                        self.oss_az_drive_d[k] = v;
                    }
                    a -= 1;
                }
                IO::OSSElDriveD { data: Some(values) } => {
                    for (k, v) in values.into_iter().enumerate() {
                        self.oss_el_drive_d[k] = v;
                    }
                    a -= 1;
                }
                IO::OSSGIRDriveD { data: Some(values) } => {
                    for (k, v) in values.into_iter().enumerate() {
                        self.oss_gir_drive_d[k] = v;
                    }
                    a -= 1;
                }
                _ => (),
            }
            if a == 0 {
                return a;
            }
            a
        }) == 0
        {
            Ok(self)
        } else {
            Err("Either mount drive controller CMD, OSSAzDriveD, OSSElDriveD or OSSGIRDriveD not found".to_owned())
        }
    }
    fn outputs(&mut self, tags: &[IO<()>]) -> Result<Option<Vec<IO<Vec<f64>>>>, String> {
        let mut check = 3;
        tags.iter()
            .fold(
                Vec::<Result<Option<IO<Vec<f64>>>, String>>::new(),
                |mut a, t| {
                    match t {
                        IO::OSSAzDriveF { .. } => {
                            a.push(Ok(Some(IO::OSSAzDriveF {
                                data: Some(Vec::<f64>::from(&self.oss_az_drive_f)),
                            })));
                            check -= 1;
                        }
                        IO::OSSElDriveF { .. } => {
                            a.push(Ok(Some(IO::OSSElDriveF {
                                data: Some(Vec::<f64>::from(&self.oss_el_drive_f)),
                            })));
                            check -= 1;
                        }
                        IO::OSSGIRDriveF { ..} => {
                            a.push(Ok(Some(IO::OSSGIRDriveF {
                                data: Some(Vec::<f64>::from(&self.oss_gir_drive_f)),
                            })));
                            check -= 1;
                        }
                        _ => (),
                    }
                    if a.len() == 3 {
                        return a;
                    }
                    a
                },
            )
            .into_iter()
            .collect::<Result<Option<Vec<_>>, _>>()
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
impl<'a> DOS<(), Vec<f64>> for ctrlr::mount::controller::Controller<'a> {
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
            if a == 0 {
                return a;
            }
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
    fn outputs(&mut self, tags: &[IO<()>]) -> Result<Option<Vec<IO<Vec<f64>>>>, String> {
        tags.iter()
            .find_map(|t| match t {
                IO::CMD { .. } => Some(IO::CMD {
                    data: Some(Vec::<f64>::from(&self.cmd)),
                }),
                _ => None,
            })
            .and_then(|x| Some(Some(vec![x])))
            .ok_or_else(|| "Unexpected DOS IO in mount controller controller".to_owned())
    }
}
//Err(),
