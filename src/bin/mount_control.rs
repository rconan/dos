use dos::{
    controllers::{m1, mount::pdr as mount, state_space::DiscreteStateSpace},
    io::jar::*,
    DataLogging, IOTags, WindLoads, DOS,
};
use fem::FEM;
use serde_pickle as pkl;
use simple_logger::SimpleLogger;
use std::error::Error;
use std::fs::File;
use std::path::Path;
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
    SimpleLogger::new().init().unwrap();
    let fem_data_path = Path::new("data").join("20210225_1447_MT_mount_v202102_ASM_wind2");
    // WIND LOADS
    let tic = Timer::tic();
    println!("Loading wind loads ...");
    let n_sample = 20 * 1000;
    let mut wind_loading = WindLoads::from_pickle(fem_data_path.join("wind_loads_2kHz.pkl"))?
        .range(0.0, 20.0)
        .decimate(2)
        .truss()?
        .m2_asm_topend()?
        .m1_segments()?
        .m1_cell()?
        .m2_asm_reference_bodies()?
        .build()?;
    tic.print_toc();
    //println!("OSS TOP END: {:?}",wind_loading.loads);
    // MOUNT CONTROL
    let mut mnt_drives = mount::drives::Controller::new();
    let mut mnt_ctrl = mount::controller::Controller::new();

    // M1
    let mut m1_hardpoints = m1::hp_load_cells::Controller::new();
    let mut m1_ctrl = m1::cg_controller::Controller::new();

    // FEM
    let sampling_rate = 1e3;
    let m1_rbm = OSSM1Lcl::new();
    let m2_rbm = MCM2RB6D::new();
    let tic = Timer::tic();
    println!("Building FEM dynamic model...");
    //fem_data_path.push("20210225_1447_MT_mount_v202102_ASM_wind2");
    //fem_data_path.set_file_name("modal_state_space_model_2ndOrder.pkl");
    let mut fem = DiscreteStateSpace::from(FEM::from_pickle(
        fem_data_path.join("modal_state_space_model_2ndOrder.pkl"),
    )?)
    .dump_eigen_frequencies(fem_data_path.join("eigen_frequencies.pkl"))
    .sampling(sampling_rate)
    .proportional_damping(2. / 100.)
    .max_eigen_frequency(75.0)
    //.eigen_frequencies(vec![(0, 1e-9), (1, 1e-9), (2, 1e-9)])
    .inputs_from(&wind_loading)
    .inputs_from(&mnt_drives)
    .outputs(vec![m1_rbm.clone(), m2_rbm.clone()])
    .outputs(vec![
        OSSAzEncoderAngle::new(),
        OSSElEncoderAngle::new(),
        OSSRotEncoderAngle::new(),
    ])
    .outputs(vec![OSSHardpointD::new()])
    .build()?;
    tic.print_toc();

    println!("FEM inputs: {:#?}", fem.inputs_tags());
    println!("FEM outputs: {:#?}", fem.outputs_tags());
    /*
    //println!("Wind loads: {:#?}", wind_loading.outputs_tags());
    println!("FEM outputs: {:#?}", fem.outputs_tags());
    println!("Mount control inputs: {:#?}", mnt_ctrl.inputs_tags());
    println!("Mount control outputs: {:#?}", mnt_ctrl.outputs_tags());
    println!("Mount drives inputs: {:#?}", mnt_drives.inputs_tags());
    println!("Mount drives outputs: {:#?}", mnt_drives.outputs_tags());
    println!("M1 load cells inputs: {:#?}", m1_hardpoints.inputs_tags());
    println!("M1 load cells outputs: {:#?}", m1_hardpoints.outputs_tags());
    println!("M1 control inputs: {:#?}", m1_ctrl.inputs_tags());
    println!("M1 control outputs: {:#?}", m1_ctrl.outputs_tags());
     */

    // DATA LOGGING
    let mut data = DataLogging::new()
        .sampling_rate(sampling_rate)
        //.key(m1_rbm.clone())
        //.key(m2_rbm.clone())
        .build();

    // OUTPUTS
    //let mut u = vec![];
    //let mut y = vec![];
    //let mut y_mnt_drive = vec![];

    //println!("Sample #: {}", wind_loading.n_sample);
    println!("Running model ...");
    let tic = Timer::tic();
    //let mut mount_drives_cmd = None;
    let mut load_cells = None;
    let mut m1_cg_fm = None;
    // FEEDBACK LOOP
    let mut mount_drives_cmd = None;
    //let mut ramp = (0..20_000).into_iter();
    let mut k = 0;
    while let Some(mut fem_forces) = wind_loading.outputs() {
        //while let Some(_) = ramp.next() {
        //  let mut fem_forces = vec![];
        data.step()?;
        // Mount Drives
        //println!("Mnt Drives: {:?}",mount_drives_cmd);
        mnt_drives
            .in_step_out(mount_drives_cmd.unwrap_or(vec![
                MountCmd::with(vec![0f64; 3]),
                OSSAzEncoderAngle::with(vec![0f64; 6]),
                OSSElEncoderAngle::with(vec![0f64; 4]),
                OSSRotEncoderAngle::with(vec![0f64; 4]),
            ]))?
            .map(|mut x| {
                fem_forces.append(&mut x);
            });
        // M1 CG Controller
        if k % 10 == 0 {
            m1_cg_fm = m1_ctrl.in_step_out(
                load_cells
                    .clone()
                    .unwrap_or(vec![M1HPLC::with(vec![0f64; 42])]),
            )?
        }
        m1_cg_fm.as_ref().map(|x| {
            fem_forces[OSSM1Lcl6F::new()] += &x[0];
            fem_forces[OSSCellLcl6F::new()] -= &x[0];
        });
        //u.push(fem_forces.clone());
        // FEM
        let ys = fem.in_step_out(fem_forces)?.ok_or("FEM output is empty")?;
        // Mount Controller
        mount_drives_cmd = mnt_ctrl.in_step_out(ys[2..5].to_vec())?.and_then(|mut x| {
            x.extend_from_slice(&ys[2..5]);
            Some(x)
        });
        // M1 HARDPOINT
        if k % 10 == 0 {
            let mut m1_hp = vec![M1HPCmd::with(vec![0f64; 42])];
            m1_hp.extend_from_slice(&[ys[5].clone()]);
            load_cells = m1_hardpoints.in_step_out(m1_hp)?;
        }
        // LOGGING
        data.log(&ys[0])?.log(&ys[1])?;
        //.log(&m1_cg_fm.as_ref().unwrap()[0])?
        //.log(&load_cells.as_ref().unwrap()[0])?;
        //.log(&ys[2])?
        //.log(&ys[3])?
        //.log(&ys[4])?
        //.log(&mount_drives_cmd.as_ref().unwrap()[0])?;
        //y.push(ys); // log
        //y_mnt_drive.push(mount_drives_cmd.as_ref().unwrap().clone()); // log
        k += 1;
    }
    tic.print_toc();

    // OUTPUTS SAVING
    //let mut f = File::create("data/wind_loads.pkl").unwrap();
    //pkl::to_writer(&mut f, &[u, y, y_mnt_drive], true).unwrap();
    //fem_data_path.set_file_name("mount_control.data.pkl");
    let mut f = File::create(fem_data_path.join("mount_control.data.pkl")).unwrap();
    //    data.time_series(m1_rbm).and_then(move |x| {
    //        data.time_series(m2_rbm).and_then(|y| Some(vec![x,y]))});
    pkl::to_writer(
        &mut f,
        &[
            data.time_series(m1_rbm),
            data.time_series(m2_rbm),
            data.time_series(M1HPLC::new()),
            data.time_series(M1CGFM::new()),
            //data.time_series(OSSAzEncoderAngle::new()),
            //data.time_series(OSSElEncoderAngle::new()),
            //data.time_series(OSSRotEncoderAngle::new()),
            //data.time_series(MountCmd::new()),
        ],
        true,
    )
    .unwrap();
    Ok(())
}
