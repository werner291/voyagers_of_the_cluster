use std::any::{Any, TypeId};
use std::borrow::BorrowMut;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, VecDeque};
use std::marker::PhantomData;
use std::ops::{Deref, DerefMut};
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
use crate::actors::Fate::{Keep, End};
use kiss3d::window::{Window, Canvas};
use std::path::Path;
use kiss3d::nalgebra::base::storage::Storage;
use kiss3d::camera::{Camera, ArcBall};
use kiss3d::event::{WindowEvent, Key, Action};
use kiss3d::nalgebra::Matrix4;
use kiss3d::resource::ShaderUniform;
use kiss3d::scene::SceneNode;

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

impl Interpretable for ScanPing {
    fn interpret(&self) -> String {
        format!("A scanner pulse bounced off something at {}", self.0)
    }
}

#[derive(Clone)]
struct ScanPulse(Point3<f64>);

impl Interpretable for ScanPulse {
    fn interpret(&self) -> String {
        format!("A scanner pulse went off at {}", self.0)
    }
}

const SCAN_PING_SPEED : f64 = 10.0;
const SCAN_PING_RANGE : f64 = 1000.0;

trait Interpretable {
    fn interpret(&self) -> String;
}

#[derive(Clone, Copy, Eq, PartialEq, Hash, Debug)]
struct ShipId(u64);

#[derive(Clone, Copy, PartialEq, Debug)]
struct ShipDestination(ShipId,Point3<f64>);

impl Interpretable for ShipDestination {
    fn interpret(&self) -> String {
        format!("Ship {:?} will travel to destination {}", self.0, self.1)
    }
}

#[derive(Clone, Copy, Eq, PartialEq, Hash, Debug)]
struct ShipArrived(ShipId);

impl Interpretable for ShipArrived {
    fn interpret(&self) -> String {
        format!("Ship {:?} has arrived at its planned destination.", self.0)
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
struct ShipMoved(ShipId,Isometry3<f64>);

impl Interpretable for ShipMoved {
    fn interpret(&self) -> String {
        format!("Ship {:?} has moved to position {}", self.0, self.1)
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
struct AsteroidCollected(Point3<f64>);

impl Interpretable for AsteroidCollected {
    fn interpret(&self) -> String {
        format!("Asteroid at {} has been collected.", self.0)
    }
}

#[derive(Clone, Copy, Eq, PartialEq, Hash, Debug)]
struct ScoreChangedTo(u64);

impl Interpretable for ScoreChangedTo {
    fn interpret(&self) -> String {
        format!("The score has been changed to {} points.", self.0)
    }
}

fn main() {

    let mut system = System::new();

    let mut rng = thread_rng();

    let mut window = Window::new_with_size("Kiss3d: cube", 1000,800);

    let asteroids = (0..10).map(|_| {
        Point3::new(
            rng.gen_range(-100.0f64 .. 100.0),
            rng.gen_range(-100.0 .. 100.0),
            rng.gen_range(-100.0 .. 100.0),
        )
    }).collect::<Vec<_>>();

    #[derive(Clone)]
    struct StartShip(ShipId);

    for ship_id in 0..5 {

        let ship_id = ShipId(ship_id);

        let starting_point = Point3::new(
            rng.gen_range(-100.0f64 .. 100.0),
            rng.gen_range(-100.0 .. 100.0),
            rng.gen_range(-100.0 .. 100.0),
        );

        struct ShipMovementController {
            position: Point3<f64>,
            destination: Option<Point3<f64>>,
            ship_id: ShipId,
        }

        system.build_actor(ShipMovementController {
            ship_id,
            position: starting_point,
            destination: None
        }).with_handler(move |state, _:&Tick, outbox| {
            if let Some(destination) = &state.destination {
                const SPEED : f64 = 0.1;
                if (destination - state.position).norm() < SPEED {
                    state.position = *destination;
                    state.destination = None;

                    outbox.send(ShipArrived(ship_id));
                } else {
                    state.position += (destination - state.position).normalize() * SPEED;
                }
            }
            outbox.send(ShipMoved(ship_id, Isometry3::translation(state.position.x,state.position.y,state.position.z)));

            Keep
        }).with_handler(move |state, ShipDestination(id, pos), outbox| {
            if *id == state.ship_id {
                state.destination = Some(*pos);
            }
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
            behavior: ShipBehavior,
            ship_id: ShipId
        }

        system.build_actor(ShipBehaviorControllerState {
            behavior: ShipBehavior::Ready,
            position: starting_point,
            ship_id
        })
            .with_handler(move |state, StartShip(id), outbox| {
                if *id == ship_id {
                    outbox.send(ScanPulse(state.position));
                    state.behavior = ShipBehavior::WaitingForPing;
                }

                Keep
            })
            .with_handler(move |state, ShipMoved(id, to), outbox| {
                if *id == state.ship_id {
                    state.position = to.translation.vector.into();
                }

                Keep
            })
            .with_handler(move |state, ScanPing(at), outbox| {
                if state.behavior == ShipBehavior::WaitingForPing {
                    state.behavior = ShipBehavior::ApproachingAsteroid(* at);

                    let delta = at - state.position;

                    outbox.send(ShipDestination(state.ship_id, at - delta.normalize() * 1.0));
                }

                Keep
            })
            .with_handler(move |state, ShipArrived(id), outbox| {
                if *id == state.ship_id {
                    if let ShipBehavior::ApproachingAsteroid(pos) = &state.behavior {
                        outbox.send(AsteroidCollected(*pos));
                        state.behavior = ShipBehavior::Ready;

                        outbox.send(delay::delay_from_now(StartShip(ship_id), Duration::from_secs(5)));
                    }
                }

                Keep
            });

        let mut miner = window.add_obj(Path::new("models/miner.obj"), Path::new("models"), Vector3::new(1.0,1.0,1.0));

        system.build_actor(miner).with_handler(move |sn,ShipMoved(id, to),_| {
            if *id == ship_id { sn.set_local_transformation(to.cast())};
            Keep
        });



        system.send(delay::delay_from_now(StartShip(ship_id), Duration::from_secs(5)));
    }

    let mut fighter = window.add_obj(Path::new("models/fighter.obj"), Path::new("models"), Vector3::new(1.0,1.0,1.0));
    fighter.set_local_translation(Translation3::new(0.0,5.0,10.0));

    system.build_actor(0)
        .with_handler(move |score, _:&AsteroidCollected, outbox| {*score += 1; outbox.send(ScoreChangedTo(*score)); Keep} );

    for pt in asteroids.iter().cloned() {
        system.build_actor(())
            .with_handler(move |sn, ScanPulse(pos), outbox| {
                outbox.send(delay::delay_from_now(ScanPing(pt), Duration::from_secs_f64((pos - pt).norm() / SCAN_PING_SPEED)));
                Keep
            })
            .with_handler(move |score, AsteroidCollected(collected_pt), outbox| {
                if collected_pt == &pt {
                    End
                } else {
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
        .with_handler(move |_,evt:&ScanPulse,_| {println!("{}", evt.interpret()); Keep})
        // .with_handler(move |_,evt:&ScanPing,_|          {println!("{}", evt.interpret()); Keep})
        .with_handler(move |_,evt:&ShipDestination,_|   {println!("{}", evt.interpret()); Keep})
        .with_handler(move |_,evt:&ShipArrived,_|       {println!("{}", evt.interpret()); Keep})
        .with_handler(move |_,evt:&AsteroidCollected, _| {println!("{}", evt.interpret()); Keep})
        .with_handler(move |_,evt:&ScoreChangedTo, _|   {println!("{}", evt.interpret()); Keep});

    let mut ringstation = window.add_obj(Path::new("models/ringstation.obj"), Path::new("models"), Vector3::new(1.0,1.0,1.0));

    system.build_actor(ringstation).with_handler(move |sn,_:&Tick,_| {

        sn.append_rotation(&UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.01));

        Keep
    });

    for roid in asteroids {
        let mut ball = window.add_sphere(1.0);
        ball.set_local_translation(roid.coords.cast().into());

        system.build_actor(ball).with_handler(move |ball, AsteroidCollected(the_roid), _| {
            if &roid == the_roid {
                ball.unlink();
                println!("Roidn't");
                End
            } else {
                Keep
            }
        });
    }

    let mut camera = kiss3d::camera::ArcBall::new(Point3::new(1.0,1.0,1.0), Point3::new(0.0,5.0,10.0));

    struct KeyState(Key, Action);

    let fighter_ship_id = ShipId(555);

    let mut camera = Rc::new(RefCell::new(camera));

    let mut camera_2 = camera.clone();

    system.build_actor(Isometry3::identity())
        .with_handler(move |xfm,KeyState(key,action),outbox| {
            match key {
                Key::Space => if *action == Action::Press { *xfm *= &Translation3::new(0.0, 0.0, -1.0); },
                Key::A => if *action == Action::Press { *xfm *= &UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.01); },
                Key::D => if *action == Action::Press { *xfm *= &UnitQuaternion::from_axis_angle(&Vector3::z_axis(), -0.01); },
                Key::W => if *action == Action::Press { *xfm *= &UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.01); },
                Key::S => if *action == Action::Press { *xfm *= &UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -0.01); },
                Key::Q => if *action == Action::Press { *xfm *= &UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.01); },
                Key::E => if *action == Action::Press { *xfm *= &UnitQuaternion::from_axis_angle(&Vector3::y_axis(), -0.01); },
                _ => {}
            }

            outbox.send(ShipMoved(fighter_ship_id, xfm.clone()));

            Keep
        });

    system.build_actor(fighter)
        .with_handler(move |sn, ShipMoved(id, at), outbox| {
            if *id == fighter_ship_id {
                sn.set_local_transformation(at.cast());
            }
            Keep
        });

    system.build_actor(camera.clone())
        .with_handler(move |cam,ShipMoved(id,at),outbox| {

            if *id == fighter_ship_id {
                let eye: Point3<f32> = (*camera_2).borrow().eye();
                let focus: Point3<f32> = at.translation.vector.cast().into();

                let new_eye_tgt: Point3<f32> = focus + (eye - focus).normalize() * 20.0;

                let new_eye: Point3<f32> = eye + (new_eye_tgt - eye) * 0.01;

                (*camera_2).borrow_mut().look_at(new_eye, focus);
            }
            Keep
        });

    let mut radar_sn = SceneNode::new_empty();

    window.scene_mut().add_child(radar_sn.clone());

    struct TrackerVizState {
        positions: HashMap<ShipId, (SceneNode, Point3<f64>)>,
        sn: SceneNode
    }

    system.build_actor(TrackerVizState {
        positions: Default::default(),
        sn: radar_sn,
    })
        .with_handler(move |st,ShipMoved(id,at),outbox| {

            if *id != fighter_ship_id {
                if st.positions.contains_key(id) {
                    st.positions.get_mut(id).unwrap().1 = at.translation.vector.into();
                } else {
                    st.positions.insert(*id, ({
                                                  let mut sn = st.sn.add_sphere(0.1);
                        sn.set_color(0.0,1.0,1.0);
                        sn
                                              }, at.translation.vector.into()));
                }
            } else {
                st.sn.set_local_translation(at.translation.cast());
            }

            Keep
        }).with_handler(move |st,_:&Tick,_| {

            let base_frame : Point3<f32> = st.sn.data().local_translation().vector.into();

            for (_,(sn,pos)) in st.positions.iter_mut() {
                sn.set_local_translation(
                    ((pos.cast()-base_frame).normalize() * 10.0).into()
                )
            }

        Keep
    });

    while window.render_with_camera((*camera).borrow_mut().deref_mut()) {

        for key in &[Key::Space,Key::Q,Key::W,Key::E,Key::A,Key::S,Key::D] {
            system.send(KeyState(*key, window.get_key(*key)));
        }

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
