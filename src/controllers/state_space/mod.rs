use crate::fem;
use crate::{io::Tags, IOTags, DOS, IO};
use log;
use nalgebra as na;
use rayon::prelude::*;
use serde_pickle as pickle;
use std::fs::File;
use std::path::Path;
use thiserror::Error;

pub mod bilinear;
pub use bilinear::Bilinear;
pub mod exponential;
pub use exponential::Exponential;

#[derive(Error, Debug)]
pub enum StateSpaceError {
    #[error("No FEM inputs match DOS {0:?}")]
    FemInputs(Tags),
    #[error("No FEM outputs match DOS {0:?}")]
    FemOutputs(Tags),
    #[error("Missing {0}")]
    MissingArguments(String),
}

type ThisResult<T> = Result<T, StateSpaceError>;
type StateSpaceIO = Option<Vec<Tags>>;

#[derive(Default)]
pub struct DiscreteStateSpace {
    sampling: Option<f64>,
    fem: Option<Box<fem::FEM>>,
    u: StateSpaceIO,
    y: StateSpaceIO,
    zeta: Option<f64>,
    eigen_frequencies: Option<Vec<(usize, f64)>>,
    max_eigen_frequency: Option<f64>,
}
impl From<fem::FEM> for DiscreteStateSpace {
    fn from(fem: fem::FEM) -> Self {
        Self {
            fem: Some(Box::new(fem)),
            ..Self::default()
        }
    }
}
impl DiscreteStateSpace {
    pub fn sampling(self, sampling: f64) -> Self {
        Self {
            sampling: Some(sampling),
            ..self
        }
    }
    pub fn proportional_damping(self, zeta: f64) -> Self {
        Self {
            zeta: Some(zeta),
            ..self
        }
    }
    pub fn eigen_frequencies(self, eigen_frequencies: Vec<(usize, f64)>) -> Self {
        Self {
            eigen_frequencies: Some(eigen_frequencies),
            ..self
        }
    }
    pub fn max_eigen_frequency(self, max_eigen_frequency: f64) -> Self {
        Self {
            max_eigen_frequency: Some(max_eigen_frequency),
            ..self
        }
    }
    pub fn dump_eigen_frequencies<P: AsRef<Path>>(self, path: P) -> Self {
        let mut file = File::create(path).unwrap();
        pickle::to_writer(
            &mut file,
            &self.fem.as_ref().unwrap().eigen_frequencies,
            true,
        )
        .unwrap();
        self
    }
    pub fn inputs(self, mut v_u: Vec<Tags>) -> Self {
        let mut u = self.u;
        if u.is_none() {
            u = Some(v_u);
        } else {
            u.as_mut().unwrap().append(&mut v_u);
        }
        Self { u, ..self }
    }
    pub fn inputs_from(self, element: &dyn IOTags) -> Self {
        self.inputs(element.outputs_tags())
    }
    pub fn outputs(self, mut v_y: Vec<Tags>) -> Self {
        let mut y = self.y;
        if y.is_none() {
            y = Some(v_y);
        } else {
            y.as_mut().unwrap().append(&mut v_y);
        }
        Self { y, ..self }
    }
    pub fn outputs_to(self, element: &dyn IOTags) -> Self {
        self.outputs(element.inputs_tags())
    }
    fn select_fem_io(fem: &mut fem::FEM, dos_inputs: &[Tags], dos_outputs: &[Tags]) {
        println!("{}", fem);
        let inputs_idx: Vec<_> = fem
            .inputs
            .iter()
            .enumerate()
            .filter_map(|(k, i)| {
                dos_inputs
                    .iter()
                    .find_map(|d| i.as_ref().and_then(|i| d.match_fem_inputs(i)).and(Some(k)))
            })
            .collect();
        let outputs_idx: Vec<_> = fem
            .outputs
            .iter()
            .enumerate()
            .filter_map(|(k, i)| {
                dos_outputs
                    .iter()
                    .find_map(|d| i.as_ref().and_then(|i| d.match_fem_outputs(i)).and(Some(k)))
            })
            .collect();
        fem.keep_inputs(&inputs_idx).keep_outputs(&outputs_idx);
        println!("{}", fem);
    }
    fn io2modes(fem: &fem::FEM, dos_inputs: &[Tags]) -> ThisResult<Vec<f64>> {
        use fem::IO;
        let indices: Vec<u32> = dos_inputs
            .iter()
            .map(|x| {
                fem.inputs
                    .iter()
                    .find_map(|y| y.as_ref().and_then(|y| x.match_fem_inputs(y)))
                    .ok_or(StateSpaceError::FemInputs(x.clone()))
            })
            .collect::<ThisResult<Vec<Vec<IO>>>>()?
            .iter()
            .flat_map(|v| {
                v.iter().filter_map(|x| match x {
                    IO::On(io) => Some(io.indices.clone()),
                    IO::Off(_) => None,
                })
            })
            .flatten()
            .collect();
        let n = fem.inputs_to_modal_forces.len() / fem.n_modes();
        Ok(fem
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
    fn modes2io(fem: &fem::FEM, dos_outputs: &[Tags]) -> ThisResult<Vec<Vec<f64>>> {
        use fem::IO;
        let n = fem.n_modes();
        let q: Vec<_> = fem.modal_disp_to_outputs.chunks(n).collect();
        Ok(dos_outputs
            .iter()
            .map(|x| {
                fem.outputs
                    .iter()
                    .find_map(|y| y.as_ref().and_then(|y| x.match_fem_outputs(y)))
                    .ok_or(StateSpaceError::FemOutputs(x.clone()))
            })
            .collect::<ThisResult<Vec<Vec<IO>>>>()?
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
    pub fn build(self) -> ThisResult<DiscreteModalSolver> {
        let tau = self.sampling.map_or(
            Err(StateSpaceError::MissingArguments("sampling".to_owned())),
            |x| Ok(1f64 / x),
        )?;
        let mut fem = self
            .fem
            .map_or(Err(StateSpaceError::MissingArguments("FEM".to_owned())), Ok)?;
        let dos_inputs = self.u.map_or(
            Err(StateSpaceError::MissingArguments("inputs".to_owned())),
            Ok,
        )?;
        let dos_outputs = self.y.map_or(
            Err(StateSpaceError::MissingArguments("outputs".to_owned())),
            Ok,
        )?;
        Self::select_fem_io(&mut fem, &dos_inputs, &dos_outputs);
        let forces_2_modes = na::DMatrix::from_row_slice(
            fem.n_modes(),
            fem.n_inputs(),
            &Self::io2modes(&fem, &dos_inputs)?,
        );
        println!("forces 2 modes: {:?}", forces_2_modes.shape());
        let fem_modes2io = Self::modes2io(&fem, &dos_outputs)?;
        let sizes: Vec<_> = fem_modes2io
            .iter()
            .map(|f| f.len() / fem.n_modes())
            .collect();
        let modes_2_nodes = na::DMatrix::from_row_slice(
            fem.n_outputs(),
            fem.n_modes(),
            &fem_modes2io.into_iter().flatten().collect::<Vec<f64>>(),
        );
        println!("modes 2 nodes: {:?}", modes_2_nodes.shape());
        let mut w = fem.eigen_frequencies_to_radians();
        if let Some(eigen_frequencies) = self.eigen_frequencies {
            log::info!("Eigen values modified");
            eigen_frequencies.into_iter().for_each(|(i, v)| {
                w[i] = v.to_radians();
            });
        }
        let n_modes = match self.max_eigen_frequency {
            Some(max_ef) => {
                fem.eigen_frequencies
                    .iter()
                    .fold(0, |n, ef| if ef <= &max_ef { n + 1 } else { n })
            }
            None => fem.n_modes(),
        };
        if let Some(max_ef) = self.max_eigen_frequency {
            log::info!("Eigen frequencies truncate to {:.3}Hz, hence reducing the number of modes from {} down to {}",max_ef,fem.n_modes(),n_modes)
        }
        let zeta = match self.zeta {
            Some(zeta) => {
                log::info!("Proportional coefficients modified, new value: {:.4}", zeta);
                vec![zeta; n_modes]
            }
            None => fem.proportional_damping_vec,
        };
        let state_space: Vec<_> = (0..n_modes)
            .map(|k| {
                let b = forces_2_modes.row(k);
                let c = modes_2_nodes.column(k);
                Exponential::from_second_order(
                    tau,
                    w[k],
                    zeta[k],
                    b.clone_owned().as_slice().to_vec(),
                    c.as_slice().to_vec(),
                )
            })
            .collect();
        Ok(DiscreteModalSolver {
            u: vec![0f64; forces_2_modes.ncols()],
            u_tags: dos_inputs,
            y: vec![0f64; modes_2_nodes.nrows()],
            y_tags: dos_outputs,
            y_sizes: sizes,
            state_space,
        })
    }
}

#[derive(Debug, Default)]
pub struct DiscreteModalSolver {
    pub u: Vec<f64>,
    u_tags: Vec<Tags>,
    pub y: Vec<f64>,
    y_sizes: Vec<usize>,
    y_tags: Vec<Tags>,
    pub state_space: Vec<Exponential>,
}
impl Iterator for DiscreteModalSolver {
    type Item = ();
    fn next(&mut self) -> Option<Self::Item> {
        let n = self.y.len();
        //        match &self.u {
        let _u_ = &self.u;
        self.y = self
            .state_space
            .par_iter_mut()
            .fold(
                || vec![0f64; n],
                |mut a: Vec<f64>, m| {
                    a.iter_mut().zip(m.solve(_u_)).for_each(|(yc, y)| {
                        *yc += y;
                    });
                    a
                },
            )
            .reduce(
                || vec![0f64; n],
                |mut a: Vec<f64>, b: Vec<f64>| {
                    a.iter_mut().zip(b.iter()).for_each(|(a, b)| {
                        *a += *b;
                    });
                    a
                },
            );
        Some(())
    }
}

impl DOS for DiscreteModalSolver {
    fn inputs(&mut self, data: Vec<IO<Vec<f64>>>) -> Result<&mut Self, Box<dyn std::error::Error>> {
        self.u = data
            .into_iter()
            .map(|x| Result::<Vec<f64>, Box<dyn std::error::Error>>::from(x))
            .collect::<Result<Vec<Vec<f64>>, Box<dyn std::error::Error>>>()?
            .into_iter()
            .flatten()
            .collect();
        Ok(self)
    }
    fn outputs(&mut self) -> Result<Option<Vec<IO<Vec<f64>>>>, Box<dyn std::error::Error>> {
        let mut pos = 0;
        self.y_tags
            .iter()
            .zip(self.y_sizes.iter())
            .map(|(t, n)| {
                let io = IO::<Vec<f64>>::from((t, self.y[pos..pos + n].to_vec()));
                pos += n;
                Ok(Some(io))
            })
            .collect()
    }
}
impl IOTags for DiscreteModalSolver {
    fn outputs_tags(&self) -> Vec<Tags> {
        self.y_tags.clone()
    }
    fn inputs_tags(&self) -> Vec<Tags> {
        self.u_tags.clone()
    }
}
