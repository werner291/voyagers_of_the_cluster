use std::any::{Any, TypeId};
use std::collections::{HashMap, VecDeque};
use std::ops::{Deref, DerefMut};
use std::rc::Rc;
use slotmap::new_key_type;
use slotmap::SlotMap;
use std::marker::PhantomData;
use crate::actors::Fate::{End, Keep};

/// Internal type referring to an entry in the SlotMap of actor states.
new_key_type! { pub struct StateKey; }

/// Struct that new messages can be placed into for later processing
pub struct MessageBox {
    queue: VecDeque<Box<dyn Any>>,
}

impl MessageBox {
    /// Enqueue a message for sending; note that the message must be sent with System::handle_one later,
    pub fn send<Msg: 'static>(&mut self, msg: Msg) {
        self.queue.push_back(Box::new(msg));
    }

    pub fn send_boxed_ugly_needsfix(&mut self, msg: Box<dyn Any>) {
        self.queue.push_back(msg);
    }
}

/// Facade struct that contains a full, working actor system.
pub struct System {
    state_store: SlotMap<StateKey, Box<dyn Any>>,
    handlers: HashMap<TypeId, Vec<(StateKey, Rc<dyn Fn(&mut dyn Any, &dyn Any, &mut MessageBox) -> Fate>)>>,
    message_box: MessageBox,
}

pub struct ActorBuilder<'a, S:'static> {
    system: &'a mut System,
    state_key: StateKey,
    phantom: PhantomData<S>
}

#[derive(Debug, Eq, PartialEq)]
pub enum Fate {
    Keep, End
}

impl<'a, S:'static> ActorBuilder<'a, S> {
    pub fn with_handler<M: 'static, F: Fn(&mut S, &M, &mut MessageBox) -> Fate + 'static>(mut self, handler: F) -> Self {
        self.system.handlers.entry(TypeId::of::<M>()).or_default().push((
            self.state_key,
            Rc::new(move |state: &mut dyn Any, message: &dyn Any, outbox: &mut MessageBox| {
                let state = state.downcast_mut::<S>().expect("Wrong state type!");
                let message = message.downcast_ref::<M>().expect("Wrong message type!");


                handler(state, message, outbox)
            })
        ));

        self
    }


}

impl System {

    /// Initialize an empty actor system
    pub fn new() -> Self {
        Self {
            state_store: SlotMap::with_key(),
            handlers: Default::default(),
            message_box: MessageBox { queue: Default::default() },
        }
    }
    pub fn build_actor<S: 'static>(&mut self, initial_state: S) -> ActorBuilder<S> {
        let state_key = self.state_store.insert(Box::new(initial_state));

        ActorBuilder {
            system: self,
            state_key,
            phantom: PhantomData::default()
        }
    }

    /// Broadcast a message into the system, to be received by all actors subscribed to that type.
    pub fn send<Msg: 'static>(&mut self, msg: Msg) {
        self.message_box.send(msg);
    }

    /// Takes one message from the queue and feeds it to the appropriate handler.
    ///
    /// Returns false if the internal queue was empty when calling the method.
    pub fn handle_one(&mut self) -> bool {

        if let Some(msg) = self.message_box.queue.pop_front() {

            if let Some(handlers) = self.handlers.get_mut(&msg.deref().type_id()) {

                let state_store = &mut self.state_store;
                let message_box = &mut self.message_box;

                handlers.retain(|(state_key, handler)| {
                    let state = state_store[*state_key].deref_mut();
                    handler(state, &*msg, message_box) == Keep
                });

                if self.handlers[&msg.deref().type_id()].is_empty() {
                    self.handlers.remove(&msg.deref().type_id());
                }
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
    use super::*;
    use crate::actors::Fate::Keep;

    #[test]
    fn hello_world() {
        struct Tick;

        let (tx, rx) = channel();

        let mut system = System::new();

        system.build_actor(0)
            .with_handler(|i, _: &Tick, outbox| {
                *i += 1;

                outbox.send(*i);

                Keep
            });

        system.build_actor(tx)
            .with_handler(|tx, i: &i32, outbox| {
                tx.send(*i).unwrap();

                Keep
            });

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