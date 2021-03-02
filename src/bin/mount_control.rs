use dos::{io::jar::*, io::IO, DOSDiscreteModalSolver, WindLoads, DOS, controllers::mount};
use fem;
use fem::{DiscreteModalSolver, FEM};
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
    //.as_outputs();
    tic.print_toc();

    let tic = Timer::tic();
    println!("Loading FEM ...");
    let mut fem = FEM::from_pickle("data/mt_fsm/modal_state_space_model_2ndOrder.pkl").unwrap();
    tic.print_toc();
    println!("{}", fem);
    fem.keep_inputs(&[1, 2, 3, 4, 5, 6, 8, 9, 10, 13])
        .keep_outputs(&[0, 1, 2, 5, 24]);
    println!("{}", fem);

    let wind_loads = vec![
        OSSCRING6F::new(),
        OSSTopEnd6F::new(),
        OSSTruss6F::new(),
        OSSGIR6F::new(),
        OSSCellLcl6F::new(),
        OSSM1Lcl6F::new(),
        MCM2Lcl6F::new(),
    ];
    let mnt_drivers = vec![OSSAzDriveF::new(), OSSElDriveF::new(), OSSGIRDriveF::new()];
    let mnt_encoders = vec![OSSAzDriveD::new(), OSSElDriveD::new(), OSSGIRDriveD::new()];

    let m1 = OSSM1Lcl::new();
    let m2 = MCM2Lcl6D::new();

    let fem_inputs: Vec<_> = wind_loads
        .iter()
        .chain(mnt_drivers.iter())
        .cloned()
        .collect();
    let mut fem_outputs: Vec<_> = vec![m1, m2]
        .iter()
        .chain(mnt_encoders.iter())
        .cloned()
        .collect();

    let mut dms = DiscreteModalSolver::new(2e3, &mut fem, &fem_inputs, &mut fem_outputs)?;
    //let mut wind_loading = WindLoading::new(&wind, &wind_loads).unwrap();

    let mut mnt_drives = mount::drives::Controller::new();
    let mut mnt_ctrl = mount::controller::Controller::new();

    println!("Sample #: {}", wind_loading.n_sample);
    println!("Running model ...");
    let tic = Timer::tic();

    let mut u = vec![];
    let mut y = vec![];
    let mut y_mnt_drive = vec![];

    let mount_cmd = vec![CMD::new()];
    let mut mount_drives_cmd: Option<Vec<IO<Vec<f64>>>> = None;

    while let Ok(Some(mut fem_forces)) = wind_loading.outputs(&wind_loads) {
        // Mount Drives
        mnt_drives
            .inputs(mount_drives_cmd.unwrap_or(vec![
                CMD::with(vec![0f64; 3]),
                OSSAzDriveD::with(vec![0f64; 8]),
                OSSElDriveD::with(vec![0f64; 8]),
                OSSGIRDriveD::with(vec![0f64; 4]),
            ]))?
            .step()?
            .outputs(&mnt_drivers)?
            .map(|mut x| {
                fem_forces.append(&mut x);
            });
        u.push(fem_forces.clone());
        // FEM
        let ys = dms
            .inputs(fem_forces)?
            .step()?
            .outputs(&fem_outputs)?
            .ok_or("FEM output is empty")?;
        // Mount Controller
        mount_drives_cmd = mnt_ctrl
            .inputs(ys[2..].to_vec())?
            .step()?
            .outputs(&mount_cmd)?
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
