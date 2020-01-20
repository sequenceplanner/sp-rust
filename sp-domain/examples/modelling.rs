use sp_domain::*;

fn main() {
    // let m = Model::new_root("model", vec![]);
    // let mut r1 = Resource::new("r1");

    // let test = MessageField::Var(Variable::new_boolean("kalle", VariableType::Measured));
    // //let n = test.as_ref();

    // let t = Topic::new("t", test);
    // println!("testing n {:?}", t);
    // //r1.add_message(t);

    // let robot_cmd = Message::new_with_type(
    //     "robot_cmd",
    //     "robot_cmd",
    //     vec![
    //         MessageField::Var(Variable::new(
    //             "ref",
    //             VariableType::Command,
    //             SPValueType::Int32,
    //             0.to_spvalue(),
    //             vec![0.to_spvalue(), 10.to_spvalue()],
    //         )),
    //         MessageField::Var(Variable::new_boolean("activate", VariableType::Command)),
    //     ],
    // );
    // let robot_state = Message::new_with_type(
    //     "robot_state",
    //     "robot_state",
    //     vec![
    //         MessageField::Var(Variable::new(
    //             "act",
    //             VariableType::Measured,
    //             SPValueType::Int32,
    //             0.to_spvalue(),
    //             vec![0.to_spvalue(), 10.to_spvalue()],
    //         )),
    //         MessageField::Var(Variable::new_boolean("active", VariableType::Measured)),
    //         MessageField::Msg(robot_cmd.instantiate("echo")),
    //     ],
    // );
    // r1.add_message(Topic::new("cmd", MessageField::Msg(robot_cmd)));
    // r1.add_message(Topic::new("state", MessageField::Msg(robot_state)));

    // let v_ref = r1
    //     .find_item("ref", &["robot_cmd"])
    //     .unwrap()
    //     .node()
    //     .local_path()
    //     .clone()
    //     .unwrap(); // this is kept here just to show what unwrap_local_path() is doing
    // let v_activate = r1.find_item("activate", &["robot_cmd"]).unwrap_local_path();
    // let v_ref_echo = r1.find_item("ref", &["echo"]).unwrap_local_path();
    // let v_activate_echo = r1.find_item("activate", &["echo"]).unwrap_local_path();
    // let v_act = r1.find_item("act", &["robot_state"]).unwrap_local_path();
    // let v_active = r1.find_item("active", &["robot_state"]).unwrap_local_path();

    // println!("v_ref: {:?}", v_ref);
    // println!("v_activate: {:?}", v_activate);
    // println!("v_ref_echo: {:?}", v_ref_echo);
    // println!("v_activate_echo: {:?}", v_activate_echo);
    // println!("v_act: {:?}", v_act);
    // println!("v_active: {:?}", v_active);

    // let name = "r1";
    // let upper = 10;
    // let to_upper = Transition::new(
    //     &format!("{}_to_upper", name),
    //     p!(v_act == 0), // p!(r != upper), // added req on v_act== 0 just for testing
    //     vec![a!(v_ref = upper)],
    //     vec![a!(v_act = upper)],
    //     false,
    // );
    // let to_lower = Transition::new(
    //     &format!("{}_to_lower", name),
    //     p!(v_act == upper), // p!(r != 0), // added req on v_act == upper just for testing
    //     vec![a!(v_ref = 0)],
    //     vec![a!(v_act = 0)],
    //     false,
    // );
    // let t_activate = Transition::new(
    //     &format!("{}_activate", name),
    //     p!(!v_active),
    //     vec![a!(v_activate)],
    //     vec![a!(v_active)],
    //     true,
    // );
    // let t_deactivate = Transition::new(
    //     &format!("{}_deactivate", name),
    //     p!(v_active),
    //     vec![a!(!v_activate)],
    //     vec![a!(!v_active)],
    //     true,
    // );

    // let _ability = Ability::new(
    //     "all",
    //     vec![t_activate, t_deactivate, to_upper, to_lower],
    //     vec![],
    // );

    // // let _ability = r1.add_ability(ability);

    // // let r1 = m.add_item(SPItem::Resource(r1)).global_path().clone().unwrap();

    // // let resource = if let Some(SPItemRef::Resource(r)) = m.get(&r1.to_sp()) {Some(r)} else {None};
    // // println!("");
    // // println!("resource: {:?}", resource);
    // // println!("");

    // // if let Some(SPItemRef::Resource(r)) = m.get(&r1.to_sp()) {
    // //     let a_again = r.get(&a.to_sp());
    // //     println!("the resource {:?}", r);
    // //     println!("the a {:?}", a_again);
    // // }

    // // let n = SPItem::Model(Model{
    // //     node: SPNode::new("n"),
    // //     items: vec!()
    // // });

    // // if let SPItem::Model(ref mut my) = m {
    // //     my.add_item(n);
    // // }

    // println!("{:?}", m);
}
