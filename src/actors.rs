use std::any::{Any, TypeId};
use std::collections::{HashMap, VecDeque};
use std::marker::PhantomData;
use std::ops::{Deref, DerefMut};
use std::rc::Rc;

use slotmap::new_key_type;
use slotmap::SlotMap;

use crate::actors::Fate::{End, Keep};

/// Internal type referring to an entry in the SlotMap of actor states.
new_key_type! { pub struct StateKey; }
new_key_type! { pub struct HandlerKey; }

pub struct SystemInterface {
    outbox: VecDeque<Box<dyn Any>>,
    new_actors: VecDeque<ActorData>,
}

impl SystemInterface {
    /// Enqueue a message for sending; note that the message must be sent with System::handle_one later,
    pub fn send<Msg: 'static>(&mut self, msg: Msg) {
        self.outbox.push_back(Box::new(msg));
    }

    pub fn send_boxed_ugly_needsfix(&mut self, msg: Box<dyn Any>) {
        self.outbox.push_back(msg);
    }

    pub fn create_actor(&mut self, actor_data: ActorData) {
        self.new_actors.push_back(actor_data);
    }
}

struct RunningActor {
    state: Box<dyn Any>,
    handlers: Vec<(TypeId, HandlerKey)>,
}

/// Facade struct that contains a full, working actor system.
pub struct System {
    state_store: SlotMap<StateKey, RunningActor>,
    handlers: HashMap<TypeId, SlotMap<HandlerKey, (StateKey, Rc<dyn Fn(&mut dyn Any, &dyn Any, &mut SystemInterface) -> Fate>)>>,
    pub input_interface: SystemInterface,
}

pub struct ActorBuilder<S> {
    init_state: S,
    handlers: Vec<(TypeId, Box<dyn Fn(&mut dyn Any, &dyn Any, &mut SystemInterface) -> Fate>)>,
}

pub struct ActorData {
    init_state: Box<dyn Any>,
    handlers: Vec<(TypeId, Box<dyn Fn(&mut dyn Any, &dyn Any, &mut SystemInterface) -> Fate>)>,
}

#[derive(Debug, Eq, PartialEq)]
pub enum Fate {
    Keep,
    End,
}

impl<S: 'static> ActorBuilder<S> {
    pub fn new(init_state: S) -> Self {
        Self {
            init_state,
            handlers: vec![],
        }
    }

    pub fn with_handler<M: 'static, F: Fn(&mut S, &M, &mut SystemInterface) -> Fate + 'static>(mut self, handler: F) -> Self {
        self.handlers.push((TypeId::of::<M>(), Box::new(move |state: &mut dyn Any, message: &dyn Any, outbox: &mut SystemInterface| {
            let state = state.downcast_mut::<S>().expect("Wrong state type!");
            let message = message.downcast_ref::<M>().expect("Wrong message type!");
            handler(state, message, outbox)
        })));
        self
    }

    pub fn build(self) -> ActorData {
        ActorData {
            init_state: Box::new(self.init_state),
            handlers: self.handlers,
        }
    }
}

impl System {
    /// Initialize an empty actor system
    pub fn new() -> Self {
        Self {
            state_store: SlotMap::with_key(),
            handlers: Default::default(),
            input_interface: SystemInterface {
                outbox: Default::default(),
                new_actors: Default::default(),
            },
        }
    }
    pub fn create_actor(&mut self, actor_data: ActorData) {
        let state_key = self.state_store.insert(RunningActor {
            state: actor_data.init_state,
            handlers: vec![],
        });

        for (typ, handler) in actor_data.handlers {
            let handler_key = self.handlers.entry(typ).or_default().insert((
                state_key, handler.into()
            ));
            self.state_store[state_key].handlers.push((typ, handler_key));
        }
    }

    /// Broadcast a message into the system, to be received by all actors subscribed to that type.
    pub fn send<Msg: 'static>(&mut self, msg: Msg) {
        self.input_interface.send(msg);
    }

    /// Takes one message from the queue and feeds it to the appropriate handler.
    ///
    /// Returns false if the internal queue was empty when calling the method.
    pub fn handle_one(&mut self) -> bool {
        if let Some(msg) = self.input_interface.outbox.pop_front() {
            if let Some(handlers) = self.handlers.get_mut(&msg.deref().type_id()) {
                let state_store = &mut self.state_store;
                let input_interface = &mut self.input_interface;

                let mut dead_actors = vec![];

                for (_, (state_key, handler)) in handlers {
                    let state = state_store[*state_key].state.deref_mut();
                    if handler(state, &*msg, input_interface) == End {
                        dead_actors.push(*state_key);
                    }
                }

                for state_key in dead_actors {
                    let corpse = self.state_store.remove(state_key).expect("Zombie handler.");
                    for (typ, hkey) in corpse.handlers {
                        self.handlers.get_mut(&typ).expect("Invalid handler reference in state store.").remove(hkey);
                        if self.handlers[&typ].is_empty() {
                            self.handlers.remove(&typ);
                        }
                    }
                }
            }

            while let Some(actor_data) = self.input_interface.new_actors.pop_front() {
                self.create_actor(actor_data);
            }

            true
        } else {
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use std::sync::mpsc::channel;

    use crate::actors::Fate::Keep;

    use super::*;

    #[test]
    fn hello_world() {
        struct Tick;

        let (tx, rx) = channel();

        let mut system = System::new();

        system.create_actor(ActorBuilder::new(0)
            .with_handler(|i, _: &Tick, outbox| {
                *i += 1;

                outbox.send(*i);

                Keep
            }).build());

        system.create_actor(ActorBuilder::new(tx)
            .with_handler(|tx, i: &i32, outbox| {
                tx.send(*i).unwrap();

                Keep
            }).build());

        system.send(Tick);
        system.send(Tick);
        system.send(Tick);
        system.send(Tick);

        while system.handle_one() {};

        assert_eq!(rx.recv().unwrap(), 1);
        assert_eq!(rx.recv().unwrap(), 2);
        assert_eq!(rx.recv().unwrap(), 3);
        assert_eq!(rx.recv().unwrap(), 4);
    }
}