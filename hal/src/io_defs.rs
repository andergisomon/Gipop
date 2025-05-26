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

/// Parses K-bus terminals and pushes them into the heap, but with `slot_idx_range` initialized to (0, 0)
pub fn parse_term(term_name: u16, term_states: Arc<RwLock<TermStates>>) {
    let guard = term_states.clone();
    let mut guard = guard.write().expect("get term_states write guard");

    log::warn!("K-bus term name: {}", term_name);

    // KL6581 is guaranteed Intelligent
    if term_name == 6581 {
        guard.kbus_terms
        .push(
            Arc::new(
                RwLock::new(
                    KBusTerm::new(
                        term_name,
                        true,
                        192,
                        KBusTerminalGender::Enby,
                        (0, 0)
                ))));
    }

    let term_name_bits: BitVec<u16, Lsb0> = BitVec::from_element(term_name as u16);

    // If Simple Terminal
    if term_name_bits[15] {
        let size_in_bits: u8 = term_name_bits[7..15].load_le();
        log::warn!("K-bus term size in bits: {}", size_in_bits);

        // If Input Terminal
        if term_name_bits[0] && !term_name_bits[1] { 
            guard.kbus_terms
            .push(
                Arc::new(
                    RwLock::new(
                        KBusTerm::new(
                            term_name,
                            false,
                            size_in_bits / 2,
                            KBusTerminalGender::Input,
                            (0, 0)
                ))));
        }

        // If Output Terminal
        if !term_name_bits[0] && term_name_bits[1] { 
            guard.kbus_terms
            .push(
                Arc::new(
                    RwLock::new(
                        KBusTerm::new(
                            term_name,
                            false,
                            size_in_bits / 2,
                            KBusTerminalGender::Output,
                            (0, 0)
                ))));
        }
    }
    log::warn!("Total K-bus terminals parsed: {}", guard.kbus_terms.len());
}

// Determine and set the correct `slot_idx_range` occupied by each K-bus terminal in the BK coupler input/output images
pub fn set_slot_idx_range(term_states: Arc<RwLock<TermStates>>) {
    let guard = term_states.clone();
    let guard = guard.write().expect("get term_states write guard");
    let terms = &guard.kbus_terms;

    // This implementation is incomplete. It does not cover the following cases:
    // - Multiple instances of the same terminal
    // - Non-contiguous terminal layout (from mixed Simple and Terminal physical layout -> cluster Simple/Terminal separately in memory).
    // TODO: KBusTerm (any terminal instance, really) should have a UID
    for (_pos, term) in terms.iter().enumerate() {
        let mut term_lock = term.write().expect("get K-bus term write guard");

        // setting slot index ranges should be conditioned on UID instead of non-unique attributes like name and gender
        if term_lock.name == 6581 {
            assert!(term_lock.intelligent && term_lock.name == 6581); // Panic if KL6581 is for some reason not Intelligent
            term_lock.slot_idx_range = (16, 15+(12*8));
        }

        if term_lock.gender == KBusTerminalGender::Input {
            term_lock.slot_idx_range = (112, 112+15);
        }

        if term_lock.gender == KBusTerminalGender::Output {
            term_lock.slot_idx_range = (112, 112+15);
        }
    }
}