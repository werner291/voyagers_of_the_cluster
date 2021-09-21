use std::any::Any;
use std::time::{Duration, Instant};

use crate::actors::System;
use crate::Tick;
use crate::actors::Fate::Keep;

pub fn delay_from_now<T:Any+'static+Clone>(ping: T, delay: Duration) -> DelayUntil {
    DelayUntil(Instant::now() + delay, Box::new(move || Box::new(ping.clone())))
}

pub struct DelayUntil(Instant, Box<dyn Fn() -> Box<dyn Any>>);

pub fn init_delay_handler(system: &mut System) {
    system.build_actor(comparator::collections::BinaryHeap::with_comparator(|a: &(Instant, Box<dyn Any>), b: &(Instant, Box<dyn Any>)| b.0.partial_cmp(&a.0).unwrap())
    ).with_handler(|st, msg: &DelayUntil, _| {
        st.push((msg.0, msg.1()));
        Keep
    }).with_handler(|st, _: &Tick, outbox| {
        while let Some(head) = st.peek() {
            if head.0 < Instant::now() {
                let msgbox : Box<dyn Any> = st.pop().unwrap().1;
                outbox.send_boxed_ugly_needsfix(msgbox);
            } else {
                break;
            }
        }
        Keep
    });
}

#[cfg(test)]
mod tests {
    use std::time::{Instant, Duration};
    use crate::actors::System;
    use crate::delay::{init_delay_handler, delay_from_now};
    use std::sync::mpsc::channel;
    use std::thread::sleep;
    use crate::Tick;

    #[test]
    fn test_delay() {

        let mut system = System::new();

        init_delay_handler(&mut system);

        #[derive(Clone)]
        struct Ping;

        let (tx,rx) = channel();

        system.build_actor(()).with_handler(move |_,_:&Ping,_| {
            tx.send(Instant::now());
        });

        let start = Instant::now();

        let target = Duration::from_secs(1);
        system.send(delay_from_now(Ping, target));

        loop {
            if let Ok(i) = rx.try_recv() {
                assert!((i.duration_since(start) - target).as_secs_f64().abs() < 0.01);
                break;
            } else {
                system.send(Tick);
                system.handle_one();
                sleep(Duration::from_millis(1));

                assert!(start.elapsed() < Duration::from_secs(2));
            }
        }

    }
}