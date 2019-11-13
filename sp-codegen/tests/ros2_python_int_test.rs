use sp_codegen::*;

#[test]
fn stuff_gen_macro() {
    generate_python_common!("macro_test_gen1");
}

#[test]
fn stuff_gen() {
    Directories::new("random_package_name_4");
    ConfigurationFile::new("random_package_name_4");
    ReadmeFile::new("random_package_name_4");
    ResourceFile::new("random_package_name_4");
    TestCopyrightFile::new("random_package_name_4");
    TestPep257File::new("random_package_name_4");
    TestFlake8File::new("random_package_name_4");
    DescriptionFile::new("random_package_name_4", "somedescr", "e@e.com", "endre");
    SetupFile::new("random_package_name_4", 
        vec!["a", "b", "c"], 
        "e@e.com", 
        "endre",
        "somedescr", 
        vec!["a", "b", "c", "asdf", "asdf2"],)
}