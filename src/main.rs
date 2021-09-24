use std::any::{Any, TypeId};
use std::borrow::BorrowMut;
use std::cell::RefCell;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, VecDeque};
use std::marker::PhantomData;
use std::ops::{Deref, DerefMut};
use std::path::Path;
use std::rc::Rc;
use std::thread::sleep;
use std::time::{Duration, Instant};

use comparator::Comparator;
use kiss3d::camera::{ArcBall, Camera};
use kiss3d::event::{Action, Key, WindowEvent};
use kiss3d::nalgebra::base::storage::Storage;
use kiss3d::nalgebra::Matrix4;
use kiss3d::resource::ShaderUniform;
use kiss3d::scene::SceneNode;
use kiss3d::window::{Canvas, Window};
use nalgebra::{distance, Quaternion, Translation3, UnitQuaternion, Vector3, Rotation3};
use nalgebra::Isometry3;
use nalgebra::Point3;
use rand::{Rng, thread_rng};
use rand::prelude::SliceRandom;
use retain_mut::RetainMut;
use slotmap::new_key_type;
use slotmap::SlotMap;
use typemap::TypeMap;

use actors::System;

use crate::actors::{ActorBuilder, ActorData, SystemInterface};
use crate::actors::Fate::{End, Keep};
use crate::delay::{delay_from_now, DelayUntil};

mod actors;
mod delay;


// struct Handler<State> {
//     state_store: StateKey,
//     handle_message: fn(&mut State, &dyn Any)
// }

struct Physics {}

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

const SCAN_PING_SPEED: f64 = 10.0;
const SCAN_PING_RANGE: f64 = 1000.0;

trait Interpretable {
    fn interpret(&self) -> String;
}

#[derive(Clone, Copy, Eq, PartialEq, Hash, Debug)]
struct ShipId(u64);

#[derive(Clone, Copy, PartialEq, Debug)]
struct ShipDestination(ShipId, Point3<f64>);

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
struct ShipMoved(ShipId, Isometry3<f64>);

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

struct ShipBehaviorControllerState {
    position: Point3<f64>,
    // This shouldn't be in here.
    behavior: ShipBehavior,
    ship_id: ShipId,
}

#[derive(PartialEq)]
enum ShipBehavior {
    Ready,
    WaitingForPing,
    ApproachingAsteroid(Point3<f64>),
}

#[derive(Copy, Clone)]
struct AsteroidCreated(Point3<f64>);

struct KeyState(Key, Action);

#[derive(Clone)]
struct StartShip(ShipId);

struct TransponderBroadcast(String, Point3<f64>);

impl Interpretable for TransponderBroadcast {
    fn interpret(&self) -> String {
        format!("Transponder with ID {} broadcast from {}.", self.0, self.1)
    }
}

fn main() {
    let mut system = System::new();

    let mut rng = thread_rng();

    let mut window = Rc::new(RefCell::new(Window::new_with_size("Kiss3d: cube", 1000, 800)));

    for _ in 0..100 {
        system.send(AsteroidCreated(Point3::new(
            rng.gen_range(-100.0f64..100.0),
            rng.gen_range(-100.0..100.0),
            rng.gen_range(-100.0..100.0),
        )));
    }

    system.create_actor(ActorBuilder::new(0).with_handler(|num, AsteroidCollected(at), outbox| {
        let mut rng = thread_rng();

        outbox.send(delay_from_now(AsteroidCreated(Point3::new(
            rng.gen_range(-100.0f64..100.0),
            rng.gen_range(-100.0..100.0),
            rng.gen_range(-100.0..100.0),
        )), Duration::from_secs(5)));

        Keep
    }).build());


    for ship_id in 0..5 {
        let ship_id = ShipId(ship_id);

        let starting_point = Point3::new(
            rng.gen_range(-100.0f64..100.0),
            rng.gen_range(-100.0..100.0),
            rng.gen_range(-100.0..100.0),
        );

        createDestinationBasedShipMovementController(&mut system, ship_id, starting_point);

        create_mining_ship_high_level_behavior_controller(&mut system, ship_id, starting_point);

        let mut miner = (*window).borrow_mut().add_obj(Path::new("models/miner.obj"), Path::new("models"), Vector3::new(1.0, 1.0, 1.0));

        system.create_actor(ActorBuilder::new(miner)
            .with_handler(move |sn, ShipMoved(id, to), _| {
                if *id == ship_id { sn.set_local_transformation(to.cast()) };
                Keep
            }).build());

        // system.create_actor(ActorBuilder::new(0).with_handler(move |t, _: &Tick, iface| {
        //     if *t == 0 {
        //         *t = 100;
        //         iface.send(TransponderBroadcast(format!("Mineral collection barge {}.", ship_id.0), Point3::new(0, 0, 0)));
        //     } else {
        //         *t -= 1;
        //     }
        //     Keep
        // }).build());

        system.send(delay::delay_from_now(StartShip(ship_id), Duration::from_secs(5)));
    }


    let mut fighter = (*window).borrow_mut().add_obj(Path::new("models/fighter.obj"), Path::new("models"), Vector3::new(1.0, 1.0, 1.0));
    fighter.set_local_translation(Translation3::new(0.0, 5.0, 10.0));

    system.create_actor(ActorBuilder::new(0)
        .with_handler(move |score, _: &AsteroidCollected, outbox| {
            *score += 1;
            outbox.send(ScoreChangedTo(*score));
            Keep
        }).build());

    let mut pirate_sn = (*window).borrow_mut().add_obj(Path::new("models/fighter.obj"), Path::new("models"), Vector3::new(1.0, 1.0, 1.0));
    pirate_sn.set_local_translation(Translation3::new(100.0, 0.0, 100.0));
    let pirate = ShipId(69);

    // createDestinationBasedShipMovementController(&mut system, pirate, Point3::new(100.0, 0.0, 100.0));

    system.create_actor(ActorBuilder::new(Isometry3::translation(100.0, 100.0, 100.0))
        .with_handler(move |position, _: &Tick, outbox| {

            let point : Point3<f64> = position.translation.vector.into();

            if point.coords.norm() > 200.0 {
                position.rotation *= &Rotation3::scaled_rotation_between(&-(position.rotation * Vector3::z_axis()), &-point.coords, 0.01).unwrap();
            }

            position.translation.vector += -(position.rotation * Vector3::z_axis()).unwrap();

            outbox.send(ShipMoved(pirate, position.clone()));

            Keep
        }).build());

    system.create_actor(ActorBuilder::new(pirate_sn).with_handler(move |sn, ShipMoved(id, at), outbox| {

        if *id == pirate {
            sn.set_local_transformation(at.cast());
        }

        Keep
    }).build());

    // system.create_actor(ActorBuilder::new(0)
    //     .with_handler(move |score, _: &AsteroidCollected, outbox| {
    //         *score += 1;
    //         outbox.send(ScoreChangedTo(*score));
    //         Keep
    //     }).build());


    // let mut sn = SceneNode::new_empty();
    // window.scene_mut().add_child(sn.clone());
    //
    // system.create_actor(AActorBuilder:new()
    //     .with_handler(|_,ScanPulse(origin),outbox| outbox.send(ExpandingSphereEffect(origin.clone())))
    //     .with_handler(|_,ScanPing(origin),outbox| outbox.send(ExpandingSphereEffect(origin.clone())));
    //
    // init_scan_pulse_visualizer(&mut system, sn);

    delay::init_delay_handler(&mut system);

    create_debug_narrator(&mut system);

    let mut ringstation = (*window).borrow_mut().add_obj(Path::new("models/ringstation.obj"), Path::new("models"), Vector3::new(1.0, 1.0, 1.0));

    system.create_actor(ActorBuilder::new(ringstation).with_handler(move |sn, _: &Tick, _| {
        sn.append_rotation(&UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.01));

        Keep
    }).build());

    system.create_actor(ActorBuilder::new(window.clone())
        .with_handler(|window, AsteroidCreated(at), system| {
            let mut ball = (**window).borrow_mut().add_sphere(1.0);
            ball.set_local_translation(at.cast().into());
            deleteScenenodeWhenAsteroidCollected(system, *at, ball);
            scanPulseResponder(system, *at);
            Keep
        }).build());

    let mut camera = kiss3d::camera::ArcBall::new(Point3::new(1.0, 1.0, 1.0), Point3::new(0.0, 5.0, 10.0));

    let fighter_ship_id = ShipId(555);

    let mut camera = Rc::new(RefCell::new(camera));

    create_keyboard_based_ship_movement_controller(&mut system.input_interface, fighter_ship_id);

    system.create_actor(ActorBuilder::new(fighter)
        .with_handler(move |sn, ShipMoved(id, at), outbox| {
            if *id == fighter_ship_id {
                sn.set_local_transformation(at.cast());
            }
            Keep
        }).build());

    createFollowcamActor(&mut system.input_interface, fighter_ship_id, &mut camera);

    let mut radar_sn = SceneNode::new_empty();

    (*window).borrow_mut().scene_mut().add_child(radar_sn.clone());

    create_ship_tracking_widget(&mut system, fighter_ship_id, radar_sn);

    while (*window).borrow_mut().render_with_camera((*camera).borrow_mut().deref_mut()) {
        for key in &[Key::Space, Key::Q, Key::W, Key::E, Key::A, Key::S, Key::D] {
            system.send(KeyState(*key, (*window).borrow_mut().get_key(*key)));
        }

        system.send(Tick);
        while system.handle_one() {}
        sleep(Duration::from_millis(10));
    }
}

fn create_ship_tracking_widget(system: &mut System, fighter_ship_id: ShipId, mut radar_sn: SceneNode) {
    system.create_actor(ActorBuilder::new(TrackerVizState {
        positions: Default::default(),
        sn: radar_sn,
    })
        .with_handler(move |st, ShipMoved(id, at), outbox| {
            if *id != fighter_ship_id {
                if st.positions.contains_key(id) {
                    st.positions.get_mut(id).unwrap().1 = at.translation.vector.into();
                } else {
                    st.positions.insert(*id, ({
                                                  let mut sn = st.sn.add_sphere(0.1);
                                                  sn.set_color(0.0, 1.0, 1.0);
                                                  sn
                                              }, at.translation.vector.into()));
                }
            } else {
                st.sn.set_local_translation(at.translation.cast());
            }

            Keep
        }).with_handler(move |st, _: &Tick, _| {
        let base_frame: Point3<f32> = st.sn.data().local_translation().vector.into();

        for (_, (sn, pos)) in st.positions.iter_mut() {
            sn.set_local_translation(
                ((pos.cast() - base_frame).normalize() * 10.0).into()
            )
        }

        Keep
    }).build());
}

struct TrackerVizState {
    positions: HashMap<ShipId, (SceneNode, Point3<f64>)>,
    sn: SceneNode,
}

fn create_debug_narrator(system: &mut System) {
    system.create_actor(ActorBuilder::new(())
        .with_handler(move |_, evt: &ScanPulse, _| {
            println!("{}", evt.interpret());
            Keep
        })
        // .with_handler(move |_,evt:&ScanPing,_|          {println!("{}", evt.interpret()); Keep})
        .with_handler(move |_, evt: &ShipDestination, _| {
            println!("{}", evt.interpret());
            Keep
        })
        .with_handler(move |_, evt: &ShipArrived, _| {
            println!("{}", evt.interpret());
            Keep
        })
        .with_handler(move |_, evt: &AsteroidCollected, _| {
            println!("{}", evt.interpret());
            Keep
        })
        .with_handler(move |_, evt: &ScoreChangedTo, _| {
            println!("{}", evt.interpret());
            Keep
        })
        .build());
}

fn scanPulseResponder(system: &mut SystemInterface, pt: Point3<f64>) {
    system.create_actor(ActorBuilder::new(())
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
        })
        .build());
}

fn deleteScenenodeWhenAsteroidCollected(system: &mut SystemInterface, roid: Point3<f64>, mut ball: SceneNode) {
    system.create_actor(ActorBuilder::new(ball).with_handler(move |ball, AsteroidCollected(the_roid), _| {
        if &roid == the_roid {
            ball.unlink();
            End
        } else {
            Keep
        }
    })
        .build());
}

fn createFollowcamActor(system: &mut SystemInterface, fighter_ship_id: ShipId, mut camera: &mut Rc<RefCell<ArcBall>>) {
    system.create_actor(ActorBuilder::new(camera.clone())
        .with_handler(move |cam, ShipMoved(id, at), outbox| {
            if *id == fighter_ship_id {
                let eye: Point3<f32> = (*cam).borrow().eye();
                let focus: Point3<f32> = at.translation.vector.cast().into();

                let new_eye_tgt: Point3<f32> = focus + (eye - focus).normalize() * 20.0;

                let new_eye: Point3<f32> = eye + (new_eye_tgt - eye) * 0.01;

                (**cam).borrow_mut().look_at(new_eye, focus);
            }
            Keep
        })
        .build());
}

fn create_keyboard_based_ship_movement_controller(system: &mut SystemInterface, fighter_ship_id: ShipId) {
    system.create_actor(ActorBuilder::new(Isometry3::identity())
        .with_handler(move |xfm, KeyState(key, action), outbox| {
            match key {
                Key::Space => if *action == Action::Press { *xfm *= &Translation3::new(0.0, 0.0, -1.0); },
                Key::A => if *action == Action::Press { *xfm *= &UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.02); },
                Key::D => if *action == Action::Press { *xfm *= &UnitQuaternion::from_axis_angle(&Vector3::z_axis(), -0.02); },
                Key::W => if *action == Action::Press { *xfm *= &UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.02); },
                Key::S => if *action == Action::Press { *xfm *= &UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -0.02); },
                Key::Q => if *action == Action::Press { *xfm *= &UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.02); },
                Key::E => if *action == Action::Press { *xfm *= &UnitQuaternion::from_axis_angle(&Vector3::y_axis(), -0.02); },
                _ => {}
            }

            outbox.send(ShipMoved(fighter_ship_id, xfm.clone()));

            Keep
        }).build());
}

fn create_mining_ship_high_level_behavior_controller(system: &mut System, ship_id: ShipId, starting_point: Point3<f64>) {
    system.create_actor(ActorBuilder::new(ShipBehaviorControllerState {
        behavior: ShipBehavior::Ready,
        position: starting_point,
        ship_id,
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
                state.behavior = ShipBehavior::ApproachingAsteroid(*at);

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
        }).build());
}

struct ShipMovementController {
    position: Point3<f64>,
    destination: Option<Point3<f64>>,
    ship_id: ShipId,
}

fn createDestinationBasedShipMovementController(system: &mut System, ship_id: ShipId, starting_point: Point3<f64>) {
    system.create_actor(ActorBuilder::new(ShipMovementController {
        ship_id,
        position: starting_point,
        destination: None,
    }).with_handler(move |state, _: &Tick, outbox| {
        if let Some(destination) = &state.destination {
            const SPEED: f64 = 0.1;
            if (destination - state.position).norm() < SPEED {
                state.position = *destination;
                state.destination = None;

                outbox.send(ShipArrived(ship_id));
            } else {
                state.position += (destination - state.position).normalize() * SPEED;
            }
        }
        outbox.send(ShipMoved(ship_id, Isometry3::translation(state.position.x, state.position.y, state.position.z)));

        Keep
    }).with_handler(move |state, ShipDestination(id, pos), outbox| {
        if *id == state.ship_id {
            state.destination = Some(*pos);
        }
        Keep
    }).build());
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
//     system.build_a ctor(ScanPulses {
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