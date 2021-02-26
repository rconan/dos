use dos::{io::jar::*, DOSDiscreteModalSolver, WindLoading, DOS};
use fem;
use fem::{DiscreteModalSolver, WindLoads, FEM};
use gmt_controllers::mount;
use serde_pickle as pkl;
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

fn main() -> Result<(), String> {
    let tic = Timer::tic();
    println!("Loading wind loads ...");
    let n_sample = 19990;
    let wind = WindLoads::from_pickle("data/trimmer_finest_mesh_20Hz.neu.pkl")
        .unwrap()
        .n_sample(n_sample);
    //.as_outputs();
    tic.print_toc();

    let tic = Timer::tic();
    println!("Loading FEM ...");
    let mut fem = FEM::from_pickle("data/modal_state_space_model_2ndOrder_v1.pkl").unwrap();
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
    let fem_inputs = vec![
        OSSCRING6F::new(),
        OSSTopEnd6F::new(),
        OSSTruss6F::new(),
        OSSGIR6F::new(),
        OSSCellLcl6F::new(),
        OSSM1Lcl6F::new(),
        MCM2Lcl6F::new(),
        OSSAzDriveF::new(),
        OSSElDriveF::new(),
        OSSGIRDriveF::new(),
    ];
    let mnt_drivers = vec![
        OSSAzDriveF::size(8),
        OSSElDriveF::size(8),
        OSSGIRDriveF::size(4),
    ];
    let mnt_encoders = vec![OSSAzDriveD::new(), OSSElDriveD::new(), OSSGIRDriveD::new()];

    let m1 = OSSM1Lcl::new();
    let m2 = MCM2Lcl6D::new();
    let mut fem_outputs = vec![m1, m2];
    fem_outputs.extend_from_slice(&mnt_encoders);
    let mut dms = DiscreteModalSolver::new(2e3, &mut fem, &fem_inputs, &mut fem_outputs)?;
    //    println!("m12: {:#?}", &fem_outputs);

    //    let mut winds = WindLoads::from_pickle("data/trimmer_finest_mesh_20Hz.neu.pkl").unwrap();
    let mut wind_loading = WindLoading::new(&wind, &wind_loads).unwrap();
    //let wind_loads = winds.to_ios(wind_loads);
    //println!("wind loads #: {}",wind_loads.len());

    let mut mnt_drives = mount::drives::Controller::new();
    let mut mnt_ctrl = mount::controller::Controller::new();

    println!("MNT CTRL: {:#?}", mnt_ctrl.cmd);

    println!("Sample #: {}", wind.n_sample.unwrap());
    println!("Running model ...");
    let tic = Timer::tic();
    let mut y = vec![];
    let mut y_mnt_drive = vec![];
    let mount_cmd = vec![CMD::size(3)];
    let mount_drives_cmd = vec![
        CMD::with(vec![0f64; 3]),
        OSSAzDriveD::with(vec![0f64; 8]),
        OSSElDriveD::with(vec![0f64; 8]),
        OSSGIRDriveD::with(vec![0f64; 4]),
    ];
    while let Ok(mut fem_forces) = wind_loading.outputs(&wind_loads) {
        fem_forces.append(
            &mut mnt_drives
                .inputs(mount_drives_cmd.clone())?
                .step()?
                .outputs(&mnt_drivers)?,
        );
        let ys = dms.inputs(fem_forces)?.step()?.outputs(&fem_outputs)?;
        let mut mount_drives_cmd = mnt_ctrl
            .inputs(ys[2..].to_vec())?
            .step()?
            .outputs(&mount_cmd)?;
        mount_drives_cmd.extend_from_slice(&ys[2..]);
        y.push(ys);
        y_mnt_drive.push(mount_drives_cmd.clone());
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
    pkl::to_writer(&mut f, &[y, y_mnt_drive], true).unwrap();
    Ok(())
}
