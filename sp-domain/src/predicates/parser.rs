use super::*;
use Predicate::*;
use PredicateValue::SPPath as PVP;
use PredicateValue::SPValue as PVV;

peg::parser!(pub grammar pred_parser() for str {
    rule _() =  quiet!{[' ' | '\t']*}

    rule path() -> SPPath =
        "p:" _ n:$(['a'..='z' | 'A'..='Z' | '0'..='9' | '_' | '/']+) {
            SPPath::from_string(n)
        }

    pub rule value() -> PredicateValue
        = _ p:path() _ { PVP(p, None) }
        / _ "true" _ { PVV(true.to_spvalue()) }
        / _ "TRUE" _ { PVV(true.to_spvalue()) }
        / _ "false" _ { PVV(false.to_spvalue()) }
        / _ "FALSE" _ { PVV(false.to_spvalue()) }
        / _ n:$(['a'..='z' | 'A'..='Z' | '_']+) _ { PVV(n.to_spvalue()) }
        / _ "\"" n:$(!['"'] [_])* "\"" _ { PVV(
            n.into_iter().collect::<Vec<_>>().join("").to_spvalue()) }
        / _ n:$(['0'..='9']+) _ { let i: i32 = n.parse().unwrap();
                              PVV(i.to_spvalue())
    }

    pub rule eq() -> Predicate
        = p1:value() _ "==" _ p2:value() { EQ(p1,p2) }
        / p1:value() _ "!=" _ p2:value() { NEQ(p1,p2) }
        / p:path() _ !['='|'!'] { EQ(PVP(p, None), PVV(true.to_spvalue())) }

    pub rule pred() -> Predicate = precedence!{
        _ p:eq() { p }
        --
            a:@ _ "->" _ b:(@) { OR(vec![NOT(Box::new(a)), b]) }
        --
            a:@ _ "||" _ b:(@) {
                match b {
                    OR(x) => {
                        let mut v = vec![a];
                        v.extend(x);
                        OR(v)
                    }
                    _ => OR(vec![a, b])
                }
            }
        --
            a:@ _ "&&" _ b:(@) {
                match b {
                    AND(x) => {
                        let mut v = vec![a];
                        v.extend(x);
                        AND(v)
                    }
                    _ => AND(vec![a, b])
                }
            }
        --
            _ "!" _ p:pred() { NOT(Box::new(p)) }
        --
            _ "(" _ p:pred() _ ")" _ { p }
        --
        _ "TRUE" _ { TRUE }
        _ "true" _ { TRUE }
        _ "FALSE" _ { FALSE }
        _ "false" _ { FALSE }
    }
});


#[test]
fn parse_values() {
    assert_eq!(pred_parser::value("9"), Ok(PredicateValue::SPValue(9.to_spvalue())));
    assert_eq!(pred_parser::value("hej"), Ok(PredicateValue::SPValue("hej".to_spvalue())));
    assert_eq!(pred_parser::value("\"hej/hopp\""), Ok(PredicateValue::SPValue("hej/hopp".to_spvalue())));
    assert_eq!(pred_parser::value("p:hej/hopp"), Ok(PredicateValue::SPPath(SPPath::from_string("hej/hopp"), None)));
    assert_eq!(pred_parser::value("p:with_underscore_and/number123"), Ok(PredicateValue::SPPath(SPPath::from_string("with_underscore_and/number123"), None)));
    assert_eq!(pred_parser::value("true"), Ok(PredicateValue::SPValue(true.to_spvalue())));
    assert_eq!(pred_parser::value("TRUE"), Ok(PredicateValue::SPValue(true.to_spvalue())));
    assert_eq!(pred_parser::value("false"), Ok(PredicateValue::SPValue(false.to_spvalue())));
    assert_eq!(pred_parser::value("left"), Ok(PredicateValue::SPValue("left".to_spvalue())));
}

#[test]
fn parse_more_tests() {
    let e = "p:cylinders2/x == left";
    let p = SPPath::from_string("cylinders2/x");
    assert_eq!(pred_parser::pred(e), Ok(p!(p:p == "left")));
}

#[test]
fn parse_predicate() {
    use Predicate::*;
    let and = "TRUE && TRUE";
    let and2 = AND(vec![TRUE,TRUE]);
    assert_eq!(pred_parser::pred(and), Ok(and2));

    let and = "TRUE  && TRUE && FALSE ";
    let and2 = AND(vec![TRUE,TRUE, FALSE]);
    assert_eq!(pred_parser::pred(and), Ok(and2));

    let or = "TRUE || TRUE || FALSE";
    let or2 = OR(vec![TRUE,TRUE, FALSE]);
    assert_eq!(pred_parser::pred(or), Ok(or2));

    let not_or = "TRUE || ! ( TRUE || FALSE && TRUE)";
    let not_or2 = OR(vec![TRUE,NOT(Box::new(OR(vec![TRUE, AND(vec![FALSE, TRUE])])))]);
    assert_eq!(pred_parser::pred(not_or), Ok(not_or2));

    let eq1 = "TRUE == TRUE";
    let eq2 = EQ(PredicateValue::SPValue(true.to_spvalue()),
                 PredicateValue::SPValue(true.to_spvalue()));
    assert_eq!(pred_parser::eq(eq1), Ok(eq2));

    let eq1 = "FALSE == p:/root/node1/node2";
    let path = SPPath::from_slice(&["root", "node1", "node2"]);
    let eq2 = EQ(PredicateValue::SPValue(false.to_spvalue()),
                 PredicateValue::SPPath(path, None));
    assert_eq!(pred_parser::eq(eq1), Ok(eq2));

    let eq1 = "p:/root/node1/node2 != false";
    let path = SPPath::from_slice(&["root", "node1", "node2"]);
    let eq2 = NEQ(PredicateValue::SPPath(path, None),
                 PredicateValue::SPValue(false.to_spvalue()));
    assert_eq!(pred_parser::eq(eq1), Ok(eq2));

    let eq1 = "TRUE == TRUE || FALSE != FALSE";
    let eq2 = EQ(PredicateValue::SPValue(true.to_spvalue()),
                 PredicateValue::SPValue(true.to_spvalue()));
    let eq3 = NEQ(PredicateValue::SPValue(false.to_spvalue()),
                  PredicateValue::SPValue(false.to_spvalue()));
    let or = OR(vec![eq2, eq3]);
    assert_eq!(pred_parser::pred(eq1), Ok(or));

    let eq1 = "TRUE == TRUE || !(FALSE != FALSE)";
    let eq2 = EQ(PredicateValue::SPValue(true.to_spvalue()),
                 PredicateValue::SPValue(true.to_spvalue()));
    let eq3 = NEQ(PredicateValue::SPValue(false.to_spvalue()),
                  PredicateValue::SPValue(false.to_spvalue()));
    let or = OR(vec![eq2, NOT(Box::new(eq3))]);
    assert_eq!(pred_parser::pred(eq1), Ok(or));


    let eq1 = "p:hej == TRUE || !(FALSE != p: hej)";
    let hej = SPPath::from_string("hej");
    let hej = PredicateValue::SPPath(hej, None);
    let eq2 = EQ(hej.clone(),
                 PredicateValue::SPValue(true.to_spvalue()));
    let eq3 = NEQ(PredicateValue::SPValue(false.to_spvalue()), hej);
    let or = OR(vec![eq2, NOT(Box::new(eq3))]);
    assert_eq!(pred_parser::pred(eq1), Ok(or));

    let impl1 = " p:  hej == TRUE ->  p: hopp == FALSE || TRUE  ";
    let hej = SPPath::from_string("hej");
    let hej = PredicateValue::SPPath(hej, None);
    let hopp = SPPath::from_string("hopp");
    let hopp = PredicateValue::SPPath(hopp, None);
    let eq1 = EQ(hej, PredicateValue::SPValue(true.to_spvalue()));
    let eq2 = EQ(hopp, PredicateValue::SPValue(false.to_spvalue()));
    let impl2 = OR(vec![NOT(Box::new(eq1)), OR(vec![eq2, TRUE])]);
    assert_eq!(pred_parser::pred(impl1), Ok(impl2.clone()));
    let impl1 = "p:hej == TRUE -> (p:hopp == FALSE || TRUE)";
    assert_eq!(pred_parser::pred(impl1), Ok(impl2));
}
