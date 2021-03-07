use crate::io::IO;
use std::collections::BTreeMap;
use thiserror::Error;

#[derive(Error, Debug)]
pub enum TellTaleError {
    #[error("Failed stepping entry index")]
    Step,
    #[error("DOS inserting entry")]
    Tale,
}

pub struct TellTale {
    pub sampling_rate: f64,
    pub keys: Vec<IO<()>>,
    pub entries: BTreeMap<usize, Vec<IO<Vec<f64>>>>,
    index: Option<usize>,
}
impl TellTale {
    pub fn step(&mut self) -> Result<&mut Self, TellTaleError>
    where
        Self: Sized + Iterator,
    {
        self.next()
            .and(Some(self))
            .ok_or_else(|| TellTaleError::Step)
    }
    pub fn tell(&mut self, tale: &IO<Vec<f64>>) -> Result<&mut Self, TellTaleError> {
        self.index
            .and_then(|i| {
                self.entries.entry(i).or_default().push(tale.clone());
                Some(())
            })
            .ok_or(TellTaleError::Tale)?;
        Ok(self)
    }
    pub fn time_series(&self, key: IO<()>) -> IO<TimeSeries> {
        let tau = self.sampling_rate.recip();
        (
            &key,
            self.entries
                .iter()
                .map(|(index, data)| {
                    data.iter()
                        .find_map(|d| if key == *d { d.into() } else { None })
                        .and_then(|x| Some((*index as f64 * tau, x)))
                })
                .collect(),
        )
            .into()
    }
}
pub type TimeSeries = Vec<(f64, Vec<f64>)>;
impl Iterator for TellTale {
    type Item = ();
    fn next(&mut self) -> Option<Self::Item> {
        self.index = self.index.map_or(Some(0), |x| Some(x + 1));
        Some(())
    }
}
pub struct DataLogging {
    pub sampling_rate: f64,
    pub keys: Vec<IO<()>>,
}
impl DataLogging {
    pub fn new() -> Self {
        Self {
            sampling_rate: 1f64,
            keys: vec![],
        }
    }
    pub fn sampling_rate(self, sampling_rate: f64) -> Self {
        Self {
            sampling_rate,
            ..self
        }
    }
    pub fn key(self, key: IO<()>) -> Self {
        let mut keys = self.keys;
        keys.push(key);
        Self { keys, ..self }
    }
    pub fn build(self) -> TellTale {
        TellTale {
            sampling_rate: self.sampling_rate,
            keys: self.keys,
            entries: BTreeMap::new(),
            index: None,
        }
    }
}
