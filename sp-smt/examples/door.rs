fn main() {
    
}

// #[test]
// fn door(){
//     let slv = slvz3!();

//     // deliberation var:
//     let delib = bvrz3!("delib");

//     // open door ability states:
//     let open_enabled = andz3!(notz3!(bvrz3!("opened_c")), notz3!(bvrz3!("opened_m")));
//     let open_executing = andz3!(bvrz3!("opened_c"), notz3!(bvrz3!("opened_m")));
//     let open_finishing = andz3!(bvrz3!("opened_c"), bvrz3!("opened_m"));
//     let open_done = andz3!(notz3!(bvrz3!("opened_c")), bvrz3!("opened_m"));

//     // close door ability states:
//     let closed_enabled = andz3!(notz3!(bvrz3!("closed_c")), notz3!(bvrz3!("closed_m")));
//     let closed_executing = andz3!(bvrz3!("closed_c"), notz3!(bvrz3!("closed_m")));
//     let closed_finishing = andz3!(bvrz3!("closed_c"), bvrz3!("closed_m"));
//     let closed_done = andz3!(notz3!(bvrz3!("closed_c")), bvrz3!("closed_m"));

//     // forbidden behavior:
//     let forb1 = notz3!(andz3!(bvrz3!("opened_c"), bvrz3!("closed_c")));
//     let forb2 = notz3!(andz3!(bvrz3!("opened_m"), bvrz3!("closed_m")));

//     // trasnitions:
//     let t1 = sasrtz3!(&slv, itez3!(&CTX, open_enabled, open_executing, open_enabled));

//     scheckz3!(&slv);
//     // println!("{}", );