//! DOS inputs/outputs
//!
//! Provides the definitions for all the inputs and outputs used by DOS

use super::{io::IO, wind_loads};
use core::fmt::Debug;

pub trait MatchFEM {
    fn match_fem_inputs(&self, fem_inputs: &fem::fem_io::Inputs) -> Option<Vec<fem::IO>>;
    fn match_fem_outputs(&self, fem_outputs: &fem::fem_io::Outputs) -> Option<Vec<fem::IO>>;
}
macro_rules! io_match_fem {
    (inputs: ($($inputs_variant:ident),+), outputs: ($($outputs_variant:ident),+)) => {
        impl<T: Debug> MatchFEM for IO<T> {
            /// Matches a FEM input to a DOS `IO` returning the FEM input value
            fn match_fem_inputs(&self, fem_inputs: &fem::fem_io::Inputs) -> Option<Vec<fem::IO>> {
                match (self,fem_inputs) {
                    $((IO::$inputs_variant{data: _}, fem::fem_io::Inputs::$inputs_variant(v)) => {
                        Some(v.clone())},)+
                    (_, _) => None,
                }
            }
            /// Matches a FEM output to a DOS `IO` returning the FEM output value
            fn match_fem_outputs(&self, fem_outputs: &fem::fem_io::Outputs) -> Option<Vec<fem::IO>> {
                match (self,fem_outputs) {
                    $((IO::$outputs_variant{data: _}, fem::fem_io::Outputs::$outputs_variant(v)) => Some(v.clone()),)+
                        (_, _) => None,
                }
            }
        }
    };
}
pub trait MatchWindLoads {
    fn data(&self, wind_loads: &wind_loads::Loads) -> Option<std::vec::IntoIter<Vec<f64>>>;
    fn ndata(
        &self,
        wind_loads: &wind_loads::Loads,
        n: usize,
    ) -> Option<std::vec::IntoIter<Vec<f64>>>;
}
macro_rules! io_match_wind_loads {
    ($($variant:ident),+) => {
        impl<T> MatchWindLoads for IO<T> {
            /// Matches a wind loads to a DOS `IO` returning the wind load value as an iterator over the first `n` elements
            fn data(&self, wind_loads: &wind_loads::Loads) -> Option<std::vec::IntoIter<Vec<f64>>> {
                match (self,wind_loads) {
                    $((IO::$variant{..}, wind_loads::Loads::$variant(v)) => Some(v.clone().into_iter()),)+
                        (_, _) => None,
                }
            }
            /// Matches a wind loads to a DOS `IO` returning the wind load value as an iterator
            fn ndata(&self, wind_loads: &wind_loads::Loads, n: usize) -> Option<std::vec::IntoIter<Vec<f64>>> {
                match (self,wind_loads) {
                    $((IO::$variant{..}, wind_loads::Loads::$variant(v)) => Some(v[..n].to_owned().into_iter()),)+
                        (_, _) => None,
                }
            }
        }
    };
}

io_match_fem!(
    inputs:
        (
            MCM2RB6F,
            MCASMCOG6F,
            MCM2TE6F,
            OSSTopEnd6F,
            OSSTruss6F,
            OSSGIR6F,
            OSSCRING6F,
            OSSCellLcl6F,
            OSSM1Lcl6F,
            MCM2Lcl6F,
            OSSAzDriveF,
            OSSElDriveF,
            OSSGIRDriveF,
            OSSHarpointDeltaF,
            OSSAzDriveTorque,
            OSSElDriveTorque,
            OSSRotDriveTorque,
            OSSM1FansLcl6F,
            OSSPayloads6F
        ),
    outputs:
        (
            OSSAzDriveD,
            OSSElDriveD,
            OSSGIRDriveD,
            OSSM1Lcl,
            MCM2Lcl6D,
            OSSHardpointD,
            OSSAzEncoderAngle,
            OSSElEncoderAngle,
            OSSRotEncoderAngle,
            MCM2RB6D,
            MCASMCOG6D,
            MCM2TE6D,
            OSSM1FansLcl6D,
            OSSPayloads6D
        )
);
io_match_wind_loads!(
    OSSTopEnd6F,
    OSSTruss6F,
    OSSGIR6F,
    OSSCRING6F,
    OSSCellLcl6F,
    OSSM1Lcl6F,
    MCM2Lcl6F
);
