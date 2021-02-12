pub trait Pairing<T, S> {
    fn pair(&mut self, other: &mut T) -> Option<S>;
}
pub trait SetU<T> {
    fn set_u(&mut self, others: &mut [T]) -> &mut Self;
}

use fem::{fem_io, wind_loads, FEM};
macro_rules! fem_pairing {
    ($($name:expr, $variant:ident),+) => {
        impl Pairing<fem_io::Inputs,Vec<f64>> for wind_loads::Outputs {
            fn pair(&mut self, fem: &mut fem_io::Inputs) -> Option<Vec<f64>> {
                match (fem,self) {
                    $((fem_io::Inputs::$variant(_),wind_loads::Outputs::$variant(v)) => {
                        v.next()
                    }),+
                        _ => None
                }
            }
        }
    };
}
fem_pairing!(
    "OSS_TopEnd_6F",
    OSSTopEnd6F,
    "OSS_Truss_6F",
    OSSTruss6F,
    "OSS_GIR_6F",
    OSSGIR6F,
    "OSS_CRING_6F",
    OSSCRING6F,
    "OSS_Cell_lcl_6F",
    OSSCellLcl6F,
    "OSS_M1_lcl_6F",
    OSSM1Lcl6F,
    "MC_M2_lcl_force_6F",
    MCM2Lcl6F
);
impl SetU<wind_loads::Outputs> for FEM {
    fn set_u(&mut self, others: &mut [wind_loads::Outputs]) -> &mut Self {
        let u: Vec<_> = self
            .inputs
            .iter_mut()
            .filter_map(|i| match i.as_mut() {
                Some(i) => others.iter_mut().filter_map(|o| o.pair(i)).next(),
                None => None,
            })
            .flatten()
            .collect();
        self.u = if u.len() > 0 { Some(u) } else { None };
        self
    }
}

use gmt_controllers::mount;
impl<'a> Pairing<mount::drives::U<'a>, ()> for mount::controller::Y<'a> {
    fn pair(&mut self, drive: &mut mount::drives::U) -> Option<()> {
        match (drive, self) {
            (mount::drives::U::CMD(d), mount::controller::Y::CMD(c)) => {
                d[0] = c[0];
                d[1] = c[1];
                d[2] = c[2];
                Some(())
            }
            _ => None,
        }
    }
}
impl<'a> Pairing<fem_io::Inputs, Vec<f64>> for mount::drives::Y<'a> {
    fn pair(&mut self, fem: &mut fem_io::Inputs) -> Option<Vec<f64>> {
        use mount::drives::Y::*;
        match (fem, self) {
            (fem_io::Inputs::OSSAzDriveF(_), OssAzDrive(v)) => Some(v.to_vec()),
            (fem_io::Inputs::OSSElDriveF(_), OssElDrive(v)) => Some(v.to_vec()),
            (fem_io::Inputs::OSSGIRDriveF(_), OssGirDrive(v)) => Some(v.to_vec()),
            _ => None,
        }
    }
}
