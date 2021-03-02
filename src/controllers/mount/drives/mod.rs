use crate::{build_controller, build_inputs, build_outputs, import_simulink, DOS, IO};

import_simulink!(MountDrives, U : (Mount_cmd,3,Mount_pos,20), Y : (Mount_F,20));
build_inputs!(
    CMD,
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
build_controller!(MountDrives,
                  U : (Mount_cmd -> (CMD,cmd) ,
                       Mount_pos -> (OssAzDrive,oss_az_drive_d),
                       Mount_pos -> (OssElDrive,oss_el_drive_d),
                       Mount_pos -> (OssGirDrive,oss_gir_drive_d)),
                  Y : (Mount_F -> (OssAzDrive,oss_az_drive_f),
                       Mount_F -> (OssElDrive,oss_el_drive_f),
                       Mount_F -> (OssGirDrive,oss_gir_drive_f))
);

// Mount
impl<'a> DOS<(), Vec<f64>> for Controller<'a> {
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
                        IO::OSSGIRDriveF { .. } => {
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
