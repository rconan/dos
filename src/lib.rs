pub mod io;
pub mod wind_loads;
pub mod controllers;


use io::IO;
pub use wind_loads::{WindLoading, WindLoads, WindLoadsError};

use fem;
use nalgebra as na;
use thiserror::Error;

#[derive(Error, Debug)]
pub enum DosError {
    #[error("Windload file not found")]
    WindLoads(#[from] WindLoadsError),
}

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
