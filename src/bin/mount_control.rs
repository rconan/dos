use dos::{
    controllers::{mount, state_space::DiscreteStateSpace},
    io::jar::*,
    WindLoads, DOS,
};
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
    // WIND LOADS
    let tic = Timer::tic();
    println!("Loading wind loads ...");
    let n_sample = 19990;
    let mut wind_loading = WindLoads::from_pickle("data/trimmer_finest_mesh_20Hz.neu.pkl")?
        .n_sample(n_sample)?
        .select_all()?
        .build()?;
    tic.print_toc();

    // MOUNT CONTROL
    let mut mnt_drives = mount::drives::Controller::new();
    let mut mnt_ctrl = mount::controller::Controller::new();

    // FEM
    let tic = Timer::tic();
    println!("Building FEM dynamic model...");
    let mut fem = DiscreteStateSpace::from(FEM::from_pickle(
        "data/mt_fsm/modal_state_space_model_2ndOrder.pkl",
    )?)
    .sampling(2e3)
    .inputs_from(&wind_loading)
    .inputs_from(&mnt_drives)
    .outputs(vec![OSSM1Lcl::new(), MCM2Lcl6D::new()])
    .outputs_to(&mnt_ctrl)
    .build()?;
    tic.print_toc();

    // OUTPUTS
    let mut u = vec![];
    let mut y = vec![];
    let mut y_mnt_drive = vec![];

    println!("Sample #: {}", wind_loading.n_sample);
    println!("Running model ...");
    let tic = Timer::tic();

    // FEEDBACK LOOP
    let mut mount_drives_cmd = None;
    while let Ok(Some(mut fem_forces)) = wind_loading.outputs() {
        // Mount Drives
        mnt_drives
            .inputs(mount_drives_cmd.unwrap_or(vec![
                MountCmd::with(vec![0f64; 3]),
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

    // OUTPUTS SAVING
    let mut f = File::create("data/wind_loads.pkl").unwrap();
    pkl::to_writer(&mut f, &[u, y, y_mnt_drive], true).unwrap();
    Ok(())
}
