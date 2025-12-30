//! Integration tests

use z3::*;

#[test]
fn test_z3_basic() {
    // Verify Z3 is working
    let cfg = Config::new();
    let ctx = Context::new(&cfg);
    let solver = Solver::new(&ctx);

    // Simple constraint: x > 0
    let x = ast::Int::new_const(&ctx, "x");
    let zero = ast::Int::from_i64(&ctx, 0);
    solver.assert(&x.gt(&zero));

    assert_eq!(solver.check(), SatResult::Sat);

    let model = solver.get_model().unwrap();
    let x_val = model.eval(&x, true).unwrap().as_i64().unwrap();
    assert!(x_val > 0);

    println!("Z3 found: x = {}", x_val);
}
