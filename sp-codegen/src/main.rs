extern crate askama; // for the Template trait and custom derive macro

use std::fs::File;
use std::io::prelude::*;
use std::io::Write;

use askama::Template; // bring trait in scope

#[derive(Template)] // this will generate the code...
#[template(path = "hello_template.html", print = "all")] // using the template in this path, relative
                                 // to the templates dir in the crate root
struct HelloTemplate<'a> { // the name of the struct can be anything
    name: &'a str, // the field name should match the variable name
                   // in your template
}
   
fn main() {
    let hello = HelloTemplate { name: "world" }; // instantiate your struct
    println!("{}", hello.render().unwrap()); // then render it.

    let lines = hello.render().unwrap();

    let mut f = File::create("/home/endre/sp-rust/sp-codegen/generated/hello.html").unwrap();
    write!(f, "{}", lines).unwrap();

}