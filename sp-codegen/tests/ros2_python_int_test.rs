use sp_codegen::*;

#[test]
fn stuff_gen() {
    Directories::new("random_package_name_4");
    ConfigurationFile::new("random_package_name_4");
    DescriptionFile::new("random_package_name_4", "somedescr", "e@e.com", "endre");
    SetupFile::new("random_package_name_4", 
        vec!["a", "b", "c"], 
        "e@e.com", 
        "endre",
        "somedescr", 
        vec!["a", "b", "c", "asdf", "asdf2"],)
}