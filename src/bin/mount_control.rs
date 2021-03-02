use dos::{
    controllers::{mount, state_space::DiscreteStateSpace},
    io::jar::*,
    io::IO,
    WindLoads, DOS,
};
use fem;
use fem::FEM;
use serde_pickle as pkl;
use std::error::Error;
use std::fs::File;
use std::time::Instant;

struct Timer {
    time: Instant,
}
impl Timer {
    pub fn tic() -> Self {
        Self {
            time: Instant::now(),
        }
    }
    pub fn toc(self) -> f64 {
        self.time.elapsed().as_secs_f64()
    }
    pub fn print_toc(self) {
        println!("... in {:3}s", self.toc());
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let tic = Timer::tic();
    println!("Loading wind loads ...");
    let n_sample = 19990;
    let mut wind_loading = WindLoads::from_pickle("data/trimmer_finest_mesh_20Hz.neu.pkl")?
        .n_sample(n_sample)?
        .select_all()?
        .build()?;
    tic.print_toc();

    let mut mnt_drives = mount::drives::Controller::new();
    let mut mnt_ctrl = mount::controller::Controller::new();

    let mut fem = {
        let tic = Timer::tic();
        println!("Loading FEM ...");
        let mut fem = FEM::from_pickle("data/mt_fsm/modal_state_space_model_2ndOrder.pkl").unwrap();
        tic.print_toc();
        println!("{}", fem);
        fem.keep_inputs(&[1, 2, 3, 4, 5, 6, 8, 9, 10, 13])
            .keep_outputs(&[0, 1, 2, 5, 24]);
        println!("{}", fem);

        DiscreteStateSpace::from(fem)
            .sampling(2e3)
            .inputs_from(&wind_loading)
            .inputs_from(&mnt_drives)
            .outputs(vec![OSSM1Lcl::new(), MCM2Lcl6D::new()])
            .outputs_to(&mnt_ctrl)
            .build()?
    };

    println!("Sample #: {}", wind_loading.n_sample);
    println!("Running model ...");
    let tic = Timer::tic();

    let mut u = vec![];
    let mut y = vec![];
    let mut y_mnt_drive = vec![];

    let mut mount_drives_cmd: Option<Vec<IO<Vec<f64>>>> = None;
    println!("Mount ctrl output: {:?}", mnt_ctrl.outputs()?);

    while let Ok(Some(mut fem_forces)) = wind_loading.outputs() {
        // Mount Drives
        mnt_drives
            .inputs(mount_drives_cmd.unwrap_or(vec![
                CMD::with(vec![0f64; 3]),
                OSSAzDriveD::with(vec![0f64; 8]),
                OSSElDriveD::with(vec![0f64; 8]),
                OSSGIRDriveD::with(vec![0f64; 4]),
            ]))?
            .step()?
            .outputs()?
            .map(|mut x| {
                fem_forces.append(&mut x);
            });
        u.push(fem_forces.clone());
        // FEM
        let ys = fem
            .inputs(fem_forces)?
            .step()?
            .outputs()?
            .ok_or("FEM output is empty")?;
        // Mount Controller
        mount_drives_cmd = mnt_ctrl
            .inputs(ys[2..].to_vec())?
            .step()?
            .outputs()?
            .and_then(|mut x| {
                x.extend_from_slice(&ys[2..]);
                Some(x)
            });
        y.push(ys); // log
        y_mnt_drive.push(mount_drives_cmd.as_ref().unwrap().clone()); // log
    }
    tic.print_toc();

    /*    let f = File::open("data/wind_loads.0.pkl").unwrap();
    let y0: Vec<Vec<f64>> = pkl::from_reader(f).unwrap();
    let y_err = y0
        .iter()
        .flatten()
        .zip(y.iter().flatten())
        .map(|(y0, y)| (y0 - y) * (y0 - y))
        .sum::<f64>()
        .sqrt();
    println!("Y ERR. : {:e}", y_err);*/
    let mut f = File::create("data/wind_loads.pkl").unwrap();
    pkl::to_writer(&mut f, &[u, y, y_mnt_drive], true).unwrap();
    Ok(())
}
