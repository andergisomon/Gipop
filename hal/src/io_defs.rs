use crate::term_cfg::*;
use bitvec::prelude::*;
use std::sync::{Arc, RwLock, LazyLock};

#[derive(Clone)]
pub struct TermStates {
    pub kbus_terms: Vec<Arc<RwLock<KBusTerm>>>,
    pub ebus_di_terms: Vec<Arc<RwLock<DITerm>>>,
    pub ebus_do_terms: Vec<Arc<RwLock<DOTerm>>>,
    pub ebus_ai_terms: Vec<Arc<RwLock<AITerm>>>,
}

// Where all the terminal states are stored dynamically on the heap
impl TermStates {
    pub fn new() -> Self {
        Self {
            kbus_terms:    Vec::new(),
            ebus_di_terms: Vec::new(),
            ebus_do_terms: Vec::new(),
            ebus_ai_terms: Vec::new(),
        }
    }
}

pub fn init_term_states() -> Arc<RwLock<TermStates>> {
    Arc::new(RwLock::new(TermStates::new()))
}