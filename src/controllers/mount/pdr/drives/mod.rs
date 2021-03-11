use crate::{
    build_controller, build_inputs, build_outputs, import_simulink,
    io::{jar, Tags},
    IOTags, DOS, IO,
};

import_simulink!(Mount_Drv_PDR2021, U : (Mount_cmd,3,Mount_pos,20), Y : (Mount_F,20));
build_inputs!(
    MountCmd,
    3,
    0,
    OssAzDrive,
    20,
    0,
    OssElDrive,
    20,
    8,
    OssGirDrive,
    20,
    16
);
build_outputs!(
    OssAzDrive,
    20,
    8,
    0,
    OssElDrive,
    20,
    8,
    8,
    OssGirDrive,
    20,
    4,
    16
);
build_controller!(Mount_Drv_PDR2021,
                  U : (Mount_cmd -> (MountCmd,cmd) ,
                       Mount_pos -> (OssAzDrive,oss_az_drive_d),
                       Mount_pos -> (OssElDrive,oss_el_drive_d),
                       Mount_pos -> (OssGirDrive,oss_gir_drive_d)),
                  Y : (Mount_F -> (OssAzDrive,oss_az_drive_f),
                       Mount_F -> (OssElDrive,oss_el_drive_f),
                       Mount_F -> (OssGirDrive,oss_gir_drive_f))
);

// Mount
impl<'a> IOTags for Controller<'a> {
    fn outputs_tags(&self) -> Vec<Tags> {
        vec![
            jar::OSSAzDriveTorque::new(),
            jar::OSSElDriveTorque::new(),
            jar::OSSRotDriveTorque::new(),
        ]
    }
    fn inputs_tags(&self) -> Vec<Tags> {
        vec![
            jar::MountCmd::new(),
            jar::OSSAzEncoderAngle::new(),
            jar::OSSElEncoderAngle::new(),
            jar::OSSRotEncoderAngle::new(),
        ]
    }
}
impl<'a> DOS for Controller<'a> {
    fn inputs(&mut self, data: Vec<IO<Vec<f64>>>) -> Result<&mut Self, Box<dyn std::error::Error>> {
        if data.into_iter().fold(4, |mut a, io| {
            match io {
                IO::MountCmd { data: Some(values) } => {
                    for (k, v) in values.into_iter().enumerate() {
                        self.cmd[k] = v;
                    }
                    a -= 1;
                }
                IO::OSSAzEncoderAngle { data: Some(values) } => {
                    for (k, v) in values.into_iter().enumerate() {
                        self.oss_az_drive_d[k] = v;
                    }
                    a -= 1;
                }
                IO::OSSElEncoderAngle { data: Some(values) } => {
                    for (k, v) in values.into_iter().enumerate() {
                        self.oss_el_drive_d[k] = v;
                    }
                    a -= 1;
                }
                IO::OSSRotEncoderAngle { data: Some(values) } => {
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
            Err("Either mount drive controller MountCmd, OSSAzEncoderAngle, OSSElEncoderAngle or OSSRotEncoderAngle not found".into())
        }
    }
    fn outputs(&mut self) -> Result<Option<Vec<IO<Vec<f64>>>>, Box<dyn std::error::Error>> {
        Ok(Some(vec![
            IO::OSSAzDriveTorque {
                data: Some(Vec::<f64>::from(&self.oss_az_drive_f)),
            },
            IO::OSSElDriveTorque {
                data: Some(Vec::<f64>::from(&self.oss_el_drive_f)),
            },
            IO::OSSRotDriveTorque {
                data: Some(Vec::<f64>::from(&self.oss_gir_drive_f)),
            },
        ]))
    }
}
