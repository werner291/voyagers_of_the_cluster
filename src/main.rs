use std::any::{Any, TypeId};
use std::borrow::BorrowMut;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, VecDeque};
use std::marker::PhantomData;
use std::ops::Deref;
use std::rc::Rc;
use std::thread::sleep;
use std::time::{Duration, Instant};

use comparator::Comparator;
use nalgebra::{Translation3, Vector3, distance, Quaternion, UnitQuaternion};
use nalgebra::Isometry3;
use nalgebra::Point3;
use rand::{Rng, thread_rng};
use rand::prelude::SliceRandom;
use retain_mut::RetainMut;
use slotmap::new_key_type;
use slotmap::SlotMap;
use typemap::TypeMap;

use actors::System;
use std::cell::RefCell;
use blue_engine::header::{Engine, WindowDescriptor};
use blue_engine::objects::triangle;
use crate::actors::Fate::{Keep, End};
use kiss3d::window::Window;
use std::path::Path;
use kiss3d::nalgebra::base::storage::Storage;

mod actors;
mod delay;


// struct Handler<State> {
//     state_store: StateKey,
//     handle_message: fn(&mut State, &dyn Any)
// }

struct Physics {

}

struct Tick;

#[derive(Clone)]
struct ScanPing(Point3<f64>);
#[derive(Clone)]
struct ScanPulse(Point3<f64>);

const SCAN_PING_SPEED : f64 = 10.0;
const SCAN_PING_RANGE : f64 = 1000.0;

fn main() {

    let mut rng = thread_rng();

    struct ShipMoved(Point3<f64>);

    let asteroids = (0..10).map(|_| {
        Point3::new(
            rng.gen_range(-100.0f64 .. 100.0),
            rng.gen_range(-100.0 .. 100.0),
            rng.gen_range(-100.0 .. 100.0),
        )
    }).collect::<Vec<_>>();

    let mut system = System::new();

    struct ShipMovementController {
        position: Point3<f64>,
        destination: Option<Point3<f64>>
    }

    #[derive(Debug)]
    struct ShipDestination(Point3<f64>);
    struct ShipArrived;

    #[derive(Debug)]
    struct CollectAsteroid(Point3<f64>);

    struct Score(u64);

    system.build_actor(0)
        .with_handler(move |score, _:&CollectAsteroid, outbox| {*score += 1; outbox.send(Score(*score)); Keep} );

    system.build_actor(ShipMovementController {
        position: Point3::origin(),
        destination: None
    }).with_handler(move |state, _:&Tick, outbox| {
        if let Some(destination) = &state.destination {
            const SPEED : f64 = 0.1;
            if (destination - state.position).norm() < SPEED {
                state.position = *destination;
                state.destination = None;

                outbox.send(ShipArrived);
            } else {
                state.position += (destination - state.position).normalize() * SPEED;
            }
        }
        outbox.send(ShipMoved(state.position));

        Keep
    }).with_handler(move |state, ShipDestination(pos), outbox| {
        eprintln!("Will go to destination: {:?}", pos.coords.data.as_slice());
        state.destination = Some(*pos);

        Keep
    });

    #[derive(PartialEq)]
    enum ShipBehavior {
        Ready,
        WaitingForPing,
        ApproachingAsteroid(Point3<f64>)
    }

    struct ShipBehaviorControllerState {
        position: Point3<f64>, // This shouldn't be in here.
        behavior: ShipBehavior
    }

    #[derive(Clone)]
    struct StartShip;

    system.build_actor(ShipBehaviorControllerState {
        behavior: ShipBehavior::Ready,
        position: Point3::origin()
    })
        .with_handler(move |state, _:&StartShip, outbox| {
            outbox.send(ScanPulse(state.position));
            state.behavior = ShipBehavior::WaitingForPing;

            Keep
        })
        .with_handler(move |state, ShipMoved(to), outbox| {
            state.position = *to;

            Keep
        })
        .with_handler(move |state, ScanPing(at), outbox| {
            if state.behavior == ShipBehavior::WaitingForPing {
                state.behavior = ShipBehavior::ApproachingAsteroid(* at);

                let delta = at - state.position;

                outbox.send(ShipDestination(at - delta.normalize() * 1.0));
            }

            Keep
        })
        .with_handler(move |state, _:&ShipArrived, outbox| {
            if let ShipBehavior::ApproachingAsteroid(pos) = &state.behavior {
                outbox.send(CollectAsteroid(*pos));
                state.behavior = ShipBehavior::Ready;

                outbox.send(delay::delay_from_now(StartShip, Duration::from_secs(5)));
            }

            Keep
        });

    system.send(delay::delay_from_now(StartShip, Duration::from_secs(5)));

    for pt in asteroids.iter().cloned() {
        system.build_actor(())
            .with_handler(move |sn, ScanPulse(pos), outbox| {
                outbox.send(delay::delay_from_now(ScanPing(pt), Duration::from_secs_f64((pos - pt).norm() / SCAN_PING_SPEED)));
                eprintln!("Asteroid {:?} echoing: ", pt.coords.data.as_slice());
                Keep
            })
            .with_handler(move |score, CollectAsteroid(collected_pt), outbox| {
                if collected_pt == &pt {
                    eprintln!("Asteroid {:?} can't be pinged.", pt.coords.data.as_slice());
                    End
                } else {
                    eprintln!("Asteroid {:?} can still be pinged", pt.coords.data.as_slice());
                    Keep
                }
            });
    }

    // let mut sn = SceneNode::new_empty();
    // window.scene_mut().add_child(sn.clone());
    //
    // system.build_actor(())
    //     .with_handler(|_,ScanPulse(origin),outbox| outbox.send(ExpandingSphereEffect(origin.clone())))
    //     .with_handler(|_,ScanPing(origin),outbox| outbox.send(ExpandingSphereEffect(origin.clone())));
    //
    // init_scan_pulse_visualizer(&mut system, sn);

    delay::init_delay_handler(&mut system);

    system.build_actor(())
        .with_handler(move |_,_:&ScanPulse,_| {println!("Scan pulse"); Keep})
        .with_handler(move |_,_:&ScanPing,_| {println!("Scan ping"); Keep})
        .with_handler(move |_,_:&ShipDestination,_| {println!("Ship moving to destination"); Keep} )
        .with_handler(move |_,_:&ShipArrived,_| {println!("Scan arrived"); Keep} )
        .with_handler(move |_,c:&CollectAsteroid,_| {println!("Asteroid collected: {:#?}",c); Keep} )
        .with_handler(move |_,Score(score),_| {println!("Score: {}", score); Keep} );

    let mut window = Window::new("Kiss3d: cube");

    let mut miner = window.add_obj(Path::new("models/miner.obj"), Path::new("models"), Vector3::new(1.0,1.0,1.0));

    system.build_actor(miner).with_handler(|sn,ShipMoved(to),_| {
        sn.set_local_translation(to.coords.cast().into());
        Keep
    });

    let mut ringstation = window.add_obj(Path::new("models/ringstation.obj"), Path::new("models"), Vector3::new(1.0,1.0,1.0));

    system.build_actor(ringstation).with_handler(move |sn,_:&Tick,_| {

        sn.append_rotation(&UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.01));

        Keep
    });

    for roid in asteroids {
        let mut ball = window.add_sphere(1.0);
        ball.set_local_translation(roid.coords.cast().into());

        system.build_actor(ball).with_handler(move |ball,CollectAsteroid(the_roid),_| {
            if &roid == the_roid {
                ball.unlink();
                End
            } else {
                Keep
            }
        });
    }

    while window.render() {
        system.send(Tick);
        while system.handle_one() {}
        sleep(Duration::from_millis(10));
    }
}
//
// #[derive(Clone)]
// struct ExpandingSphere {
//     // sn: SceneNode,
//     time: Instant
// }
//
// #[derive(Clone)]
// struct ScanPulses {
//     pulses: Vec<ExpandingSphere>,
//     parent_sn: SceneNode
// }
//
// #[derive(Clone)]
// struct ExpandingSphereEffect(Point3<f64>);
//
// fn init_scan_pulse_visualizer(system: &mut System, mut sn: SceneNode) {
//
//     system.build_actor(ScanPulses {
//         pulses: Vec::new(),
//         parent_sn: sn
//     })
//         .with_handler(move |st, ExpandingSphereEffect(pos), outbox| {
//             let mut node = st.parent_sn.add_sphere(0.0);
//             node.set_local_translation(pos.cast().into());
//             node.set_color(0.0, 1.0, 1.0);
//
//             st.pulses.push(ExpandingSphere {
//                 sn: node,
//                 time: Instant::now()
//             })
//         })
//         .with_handler(move |st, _: &Tick, outbox| {
//             st.pulses.retain_mut(|es| {
//                 let age = Instant::now().duration_since(es.time);
//
//                 let age_t = age.as_secs_f64();
//
//                 if age_t >= SCAN_PING_RANGE / SCAN_PING_SPEED {
//                     es.sn.unlink();
//                     false
//                 } else {
//                     es.sn.set_local_scale(
//                         (age_t * SCAN_PING_SPEED) as f32,
//                         (age_t * SCAN_PING_SPEED) as f32,
//                         (age_t * SCAN_PING_SPEED) as f32
//                     );
//                     true
//                 }
//             });
//         });
// }
