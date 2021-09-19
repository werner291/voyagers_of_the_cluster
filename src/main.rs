use std::any::{Any, TypeId};
use std::borrow::BorrowMut;
use std::collections::{HashMap, VecDeque, BinaryHeap};
use std::marker::PhantomData;
use std::ops::Deref;
use std::rc::Rc;
use std::thread::sleep;
use std::time::{Duration, Instant};

use kiss3d::light::Light;
use kiss3d::window::Window;
use nalgebra::{Translation3, Vector3};
use nalgebra::Isometry3;
use nalgebra::Point3;
use rand::{Rng, thread_rng};
use rand::prelude::SliceRandom;
use slotmap::new_key_type;
use slotmap::SlotMap;
use typemap::TypeMap;

use actors::System;
use kiss3d::scene::SceneNode;
use retain_mut::RetainMut;
use std::cmp::Ordering;
use comparator::Comparator;

mod actors;


// struct Handler<State> {
//     state_store: StateKey,
//     handle_message: fn(&mut State, &dyn Any)
// }

struct Physics {

}

struct Tick;

trait Message : Clone {}

impl<M:Clone> Message for M {}

fn main() {

    let mut window = Window::new("Kiss3d: cube");
    window.set_background_color(0.0,0.0,0.0);

    let mut rng = thread_rng();

    #[derive(Clone)]
    struct ShipState {
        position: Isometry3<f64>
    }

    struct ShipMoved(Isometry3<f64>);

    let asteroids = (0..10).map(|_| {
        Point3::new(
            rng.gen_range(-100.0f64 .. 100.0),
            rng.gen_range(-100.0 .. 100.0),
            rng.gen_range(-100.0 .. 100.0),
        )
    }).collect::<Vec<_>>();

    for asteroid in &asteroids {
        let mut s = window.add_sphere(0.5);
        s.set_local_translation(Translation3::from(asteroid.coords.cast()));
    }

    let mut c = window.add_cube(1.0,1.0,1.0);

    window.set_light(Light::StickToCamera);

    let mut system = System::new();

    system.build_actor(ShipState {
        position: Isometry3::identity()
    }).with_handler(move |state, _:&Tick, outbox| {
        state.position.translation.y += 0.1;
        outbox.send(ShipMoved(state.position.cast()));
    });

    system.build_actor(c)
        .with_handler(move |sn, ShipMoved(pos), outbox| {
        sn.set_local_transformation(pos.cast());
    });

    struct ScanPulse(Point3<f64>);
    struct ScanPing(Point3<f64>);

    #[derive(Clone)]
    struct ExpandingSphere {
        sn: SceneNode,
        time: Instant
    }


    system.send(ScanPulse(Point3::new(0.0,0.0,0.0)));

    for pt in asteroids.iter().cloned() {
        system.build_actor(())
            .with_handler(move |sn, ScanPulse(pos), outbox| {
                outbox.send(ScanPing(pt));
            });
    }

    let mut sn = SceneNode::new_empty();
    window.scene_mut().add_child(sn.clone());

    #[derive(Clone)]
    struct ScanPulses {
        pulses: Vec<ExpandingSphere>,
        parent_sn: SceneNode
    }

    system.build_actor(ScanPulses {
        pulses: Vec::new(),
        parent_sn: sn
    })
        .with_handler(move |st, ScanPulse(pos), outbox| {
            let mut node = st.parent_sn.add_sphere(0.0);
            node.set_local_translation(pos.cast().into());
            node.set_color(0.0, 1.0, 1.0);
            st.pulses.push(ExpandingSphere {
                sn: node,
                time: Instant::now()
            })
        })
        .with_handler(move |st, _:&Tick, outbox| {
            st.pulses.retain_mut(|es| {
                let age = Instant::now().duration_since(es.time);

                let age_t = age.as_secs_f64() / 2.0;

                if age_t >= 1.0 {
                    es.sn.unlink();
                    false
                } else {
                    es.sn.set_local_scale(
                        (age_t * 100.0) as f32,
                        (age_t * 100.0) as f32,
                        (age_t * 100.0) as f32
                    );
                    true
                }
            });
        });

    struct DelayUntil(Instant, Box<dyn Fn() -> Box<dyn Any>>);

    system.build_actor(comparator::collections::BinaryHeap::with_comparator(|a:&(Instant, Box<dyn Any>),b:&(Instant, Box<dyn Any>)| a.0.partial_cmp(&b.0).unwrap())
    ).with_handler(|st, msg: &DelayUntil, _| {
        st.push((msg.0, msg.1()));
    }).with_handler(|st, _:&Tick,outbox| {
        while let Some(head) = st.peek() {
            if head.0 < Instant::now() {
                outbox.send(st.pop().unwrap().1);
            } else {
                break;
            }
        }
    });

    while window.render() {
        system.send(Tick);
        while system.handle_one() {};
    }
}
