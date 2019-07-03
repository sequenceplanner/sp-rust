


#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub enum SOP {
    Sequence { spid: SPID, sop: Vec<SOP> },
    Parallel { spid: SPID, sop: Vec<SOP> },
    Alternative { spid: SPID, sop: Vec<SOP> },
    Arbitrary { spid: SPID, sop: Vec<SOP> },
    Other { spid: SPID, sop: Vec<SOP> },
    Operation { op: Uuid, spid: SPID, sop: Vec<SOP> },
}

impl Default for SOP {
    fn default() -> Self {
        SOP::Parallel {
            spid: SPID::default(),
            sop: Vec::new(),
        }
    }
}