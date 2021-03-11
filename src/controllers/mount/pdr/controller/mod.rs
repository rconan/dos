use crate::{
    build_controller, build_inputs, build_outputs, import_simulink,
    io::{jar, Tags},
    IOTags, DOS, IO,
};

import_simulink!(Mount_Control, U : (Mount_SP,3,Mount_FB,20), Y : (Mount_cmd,3));
build_inputs!(
    SP,
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
build_outputs!(MountCmd, 3);
build_controller!(Mount_Control,
                  U : (Mount_FB -> (OssAzDrive,oss_az_drive),
                       Mount_FB -> (OssElDrive,oss_el_drive),
                       Mount_FB -> (OssGirDrive,oss_gir_drive)),
                  Y : (Mount_cmd -> (MountCmd,cmd))
);

impl<'a> IOTags for Controller<'a> {
    fn outputs_tags(&self) -> Vec<Tags> {
        vec![jar::MountCmd::new()]
    }
    fn inputs_tags(&self) -> Vec<Tags> {
        vec![
            jar::OSSAzDriveD::new(),
            jar::OSSElDriveD::new(),
            jar::OSSGIRDriveD::new(),
        ]
    }
}
impl<'a> DOS for Controller<'a> {
    fn inputs(&mut self, data: Vec<IO<Vec<f64>>>) -> Result<&mut Self, Box<dyn std::error::Error>> {
        if data.into_iter().fold(3, |mut a, io| {
            match io {
                IO::OSSAzEncoderAngle { data: Some(values) } => {
                    for (k, v) in values.into_iter().enumerate() {
                        self.oss_az_drive[k] = v;
                    }
                    a -= 1;
                }
                IO::OSSElEncoderAngle { data: Some(values) } => {
                    for (k, v) in values.into_iter().enumerate() {
                        self.oss_el_drive[k] = v;
                    }
                    a -= 1;
                }
                IO::OSSRotEncoderAngle { data: Some(values) } => {
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
            Err("Either mount controller controller OSSAzDriveD, OSSElDriveD or OSSGIRDriveD not found".into())
        }
    }
    fn outputs(&mut self) -> Result<Option<Vec<IO<Vec<f64>>>>, Box<dyn std::error::Error>> {
        Ok(Some(vec![IO::MountCmd {
            data: Some(Vec::<f64>::from(&self.cmd)),
        }]))
    }
}
