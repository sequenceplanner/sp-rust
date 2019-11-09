# sp-smt

High-level rust bindings to the Z3 theorem prover with a completely macro based interface.

Z3 is a SMT solver from Microsoft Research which supports the SMT-LIB format.

Go to the documentation with:
```
cargo doc --open
```
since time has been spent to actually write it.

Look at `lib.rs` to find the list of currently implemented interfaces.

Look in `tests` to find examples of how to use mathods and macros.

Run a specific test with:
```
cargo test -- --test <test_name>
```

Look in `examples` to find more comprehensive examples.

Run a specific example with:
```
cargo run --example <example_name>
```

Don't look in `experiments`.