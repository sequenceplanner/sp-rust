//! The SPPath is used for idetifying items in the model

use super::*;


/// The SPPath is used for identifying all objects in a model. The path will be defined
/// based on where the item is in the model hierarchy
#[derive(Hash, Eq, PartialEq, PartialOrd, Ord, Serialize, Deserialize, Clone)]
pub enum SPPath {
    LocalPath(LocalPath),
    GlobalPath(GlobalPath),
}

impl Default for SPPath {
    fn default() -> Self {
        SPPath::LocalPath(LocalPath{path: vec!()})
    }
}

impl std::fmt::Display for SPPath {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            SPPath::LocalPath(x) => write!(f, "{}", x),
            SPPath::GlobalPath(x) => write!(f, "{}", x),
        }
    }
}
impl std::fmt::Debug for SPPath {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            SPPath::LocalPath(x) => write!(f, "{}", x),
            SPPath::GlobalPath(x) => write!(f, "{}", x),
        }
    }
}

impl SPPath {
    pub fn new_local() -> SPPath {
        SPPath::LocalPath(LocalPath::new())
    }
    pub fn new_global() -> SPPath {
        SPPath::GlobalPath(GlobalPath::new())
    }
    pub fn add(&mut self, name: String) {
        match self {
            SPPath::LocalPath(ref mut xs) => xs.add(name),
            SPPath::GlobalPath(ref mut xs) => xs.add(name)
        }
    }
    pub fn add_root(&mut self, name: String) {
        match self {
            SPPath::LocalPath(ref mut xs) => xs.add_root(name),
            SPPath::GlobalPath(ref mut xs) => xs.add_root(name)
        }
    }
    pub fn from(xs: &[String]) -> SPPath {
        let v: Vec<String> = xs.iter().map(|s| s.to_string()).collect();
        SPPath::LocalPath(LocalPath::from(v))
    }
    pub fn from_array(xs: &[&str]) -> SPPath {
        let v: Vec<String> = xs.iter().map(|s| s.to_string()).collect();
        SPPath::LocalPath(LocalPath::from(v))
    }
    pub fn from_string(s: &str) -> Result<SPPath> {
        let what_type: Vec<&str> = s.split(":").collect();

        match what_type.as_slice() {
            ["L", tail] => {
                let res: Vec<&str> = tail.split("/").collect();
                return Ok(SPPath::from_array(&res));
            }
            ["G", tail] if tail != &"" => {
                let res: Vec<&str> = tail.split("/").filter(|x| !x.is_empty()).collect();
                return Ok(SPPath::from_array_to_global(&res));
            }
            _ => return Err(SPError::No(format!("Can not convert {} into a path", s))),
        }
    }
    pub fn from_to_global(n: &[String]) -> SPPath {
        let v: Vec<String> = n.iter().map(|s| s.to_string()).collect();
        SPPath::GlobalPath(GlobalPath::from(v))
    }
    pub fn from_array_to_global(n: &[&str]) -> SPPath {
        let v: Vec<String> = n.iter().map(|s| s.to_string()).collect();
        SPPath::GlobalPath(GlobalPath::from(v))
    }
    pub fn path(&self) -> Vec<String> {
        match self {
            SPPath::LocalPath(x) => x.path(),
            SPPath::GlobalPath(x) => x.path(),
        }
    }
    pub fn string_path(&self) -> String {
        format!("{}", self)
    }
    pub fn is_child_of(&self, other: &SPPath) -> bool {
        (self.path().len() > other.path().len())
            && other
                .path()
                .iter()
                .zip(self.path().iter())
                .all(|(a, b)| a == b)
    }

    /// returns the next name in the path of this SPPath based on a path
    /// that is the current parent to this path
    pub fn next_node_in_path(&self, parent_path: &SPPath) -> Option<String> {
        if self.is_child_of(parent_path) && self.as_slice().len() > parent_path.as_slice().len() {
            Some(self.as_slice()[parent_path.as_slice().len()].clone())
        } else {
            None
        }
    }

    pub fn as_sp(&self) -> SPPath {
        self.clone()
    }

    /// For internal use instead of cloning the path vec
    pub fn as_slice(&self) -> &[String] {
        match self {
            SPPath::LocalPath(x) => x.as_slice(),
            SPPath::GlobalPath(x) => x.as_slice(),
        }
    }
}

// Maybe refactor into a trait with methods that are the same
// pub trait Pather {
//     fn path(&self) -> &[String];
//     fn path_mut(&mut self) -> &mut [String];
// }

#[derive(Hash, Eq, PartialEq, PartialOrd, Ord, Serialize, Deserialize, Clone, Default)]
pub struct LocalPath {
    path: Vec<String>,
}
impl LocalPath {
    pub fn new() -> LocalPath {
        LocalPath{path: vec!()}
    }
    pub fn path(&self) -> Vec<String> {
        self.path.clone()
    }
    pub fn from(path: Vec<String>) -> LocalPath {
        LocalPath{path}
    }
    pub fn add(&mut self, name: String) {
        self.path.push(name);
    }
    pub fn add_root(&mut self, name: String) {
        self.path.insert(0,name);
    }
    pub fn as_slice(&self) -> &[String] {
        self.path.as_slice()
    }
    pub fn as_sp(&self) -> SPPath {
        SPPath::LocalPath(self.clone())
    }
    pub fn is_child_of(&self, other: &LocalPath) -> bool {
        (self.as_slice().len() > other.as_slice().len())
            && other
                .as_slice()
                .iter()
                .zip(self.as_slice().iter())
                .all(|(a, b)| a == b)
    }
}
impl std::fmt::Display for LocalPath {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "L:{}", self.path.join("/"))
    }
}
impl std::fmt::Debug for LocalPath {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "L:{}", self.path.join("/"))
    }
}

#[derive(Hash, Eq, PartialEq, PartialOrd, Ord, Serialize, Deserialize, Clone, Default)]
pub struct GlobalPath{
    path: Vec<String>,
}
impl GlobalPath {
    pub fn new() -> GlobalPath {
        GlobalPath{path: vec!()}
    }
    pub fn path(&self) -> Vec<String> {
        self.path.clone()
    }
    pub fn from(path: Vec<String>) -> GlobalPath {
        GlobalPath{path}
    }
    pub fn add(&mut self, name: String) {
        self.path.push(name);
    }
    pub fn add_root(&mut self, name: String) {
        self.path.insert(0,name);
    }
    pub fn as_slice(&self) -> &[String] {
        self.path.as_slice()
    }
    pub fn as_sp(&self) -> SPPath {
        SPPath::GlobalPath(self.clone())
    }
    pub fn is_child_of(&self, other: &GlobalPath) -> bool {
        (self.as_slice().len() > other.as_slice().len())
            && other
                .as_slice()
                .iter()
                .zip(self.as_slice().iter())
                .all(|(a, b)| a == b)
    }
}
impl std::fmt::Display for GlobalPath {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "G:{}", self.path.join("/"))
    }
}
impl std::fmt::Debug for GlobalPath {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self)
    }
}


#[derive(Debug, Hash, Eq, PartialEq, PartialOrd, Ord, Serialize, Deserialize, Clone, Default)]
pub struct SPPaths {
    local: Option<LocalPath>,
    global: Option<GlobalPath>,
}

impl std::fmt::Display for SPPaths {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        let l: String = self.local.as_ref().map(|x| format!("{}", x)).unwrap_or("".to_string());
        let g: String = self.global.as_ref().map(|x| format!("{}", x)).unwrap_or("".to_string());
        write!(f, "<{},{}>", l, g)
    }
}

impl SPPaths {
    pub fn new(local: Option<LocalPath>, global: Option<GlobalPath>) -> SPPaths {
        SPPaths {
            local,
            global
        }
    }
    pub fn empty() -> SPPaths {
        SPPaths::default()
    }
    pub fn local_path(&self) -> &Option<LocalPath> {
        &self.local
    }
    pub fn global_path(&self) -> &Option<GlobalPath> {
        &self.global
    }

    pub fn is_eq(&self, path: &SPPath) -> bool {
        match path {
            SPPath::LocalPath(p) => self.local.as_ref().map(|l| l == p).unwrap_or(false),
            SPPath::GlobalPath(p) => self.global.as_ref().map(|g| g == p).unwrap_or(false),
        }
    }
    pub fn is_parent_of(&self, path: &SPPath) -> bool {
        match path {
            SPPath::LocalPath(p) => self.local.as_ref().map(|x| p.is_child_of(&x)).unwrap_or(false),
            SPPath::GlobalPath(p) => self.global.as_ref().map(|x| p.is_child_of(&x)).unwrap_or(false),
        }
    }

    /// returns the next name in the path based on a local or gloabl path
    /// in this SPPaths
    pub fn next_node_in_path(&self, path: &SPPath) -> Option<String> {
        let l_len: usize = self.local.as_ref().map(|x| x.as_slice().len()).unwrap_or(0);
        let g_len: usize = self.global.as_ref().map(|x| x.as_slice().len()).unwrap_or(0);
        if self.is_parent_of(path){
            match path {
                SPPath::LocalPath(x) =>  {
                    Some(x.as_slice()[l_len].clone())
                },
                SPPath::GlobalPath(x) => {
                    Some(x.as_slice()[g_len].clone())
                }
            }      
        } else {
            None
        }
    }


    pub fn upd(&mut self, paths: &SPPaths) {
        self.local = paths.local.clone();
        self.global = paths.global.clone();
    }
    pub fn add(&mut self, name: String) {
        self.local.as_mut().map(|x| x.path.push(name.clone()));
        self.global.as_mut().map(|x| x.path.push(name));
    }
    pub fn upd_local(&mut self, path: Option<LocalPath>) {
        self.local = path;
    }
    pub fn upd_global(&mut self, path: Option<GlobalPath>) {
        self.global = path;
    }
}



#[cfg(test)]
mod tests_paths {
    use super::*;
    #[test]
    fn making() {
        let g_ab = SPPath::from_array_to_global(&["a", "b"]);
        let l_ab = SPPath::from_array(&["a", "b"]);

        assert_eq!(g_ab.string_path(), "G:a/b".to_string());
        assert_eq!(l_ab.string_path(), "L:a/b".to_string());
        assert_eq!(SPPath::from_string("G:a/b"), Ok(g_ab.clone()));
        assert_eq!(SPPath::from_string("L:a/b"), Ok(l_ab.clone()));

        assert!(SPPath::from_string("G:").is_err());
        assert!(SPPath::from_string("G/no").is_err());
        assert!(SPPath::from_string("H:a/b/").is_err());
        assert!(SPPath::from_string("a/b/").is_err());
        assert!(SPPath::from_string("G:top_path").is_ok());
        assert_eq!(
            SPPath::from_string("G:a/b//k/"),
            Ok(SPPath::from_array_to_global(&["a", "b", "k"]))
        );
    }

    #[test]
    fn get_next_name() {
        let g_ab = SPPath::from_array_to_global(&["a", "b", "c"]);
        let l_ab = SPPath::from_array(&["a", "b", "c"]);

        let l_a = SPPath::from_array(&["a"]);
        let l_b = SPPath::from_array(&["a", "b"]);
        let g_a = SPPath::from_array_to_global(&["a"]);
        let g_b = SPPath::from_array_to_global(&["a", "b"]);
        let l_k = SPPath::from_array(&["k"]);

        assert_eq!(g_ab.next_node_in_path(&l_a), Some("b".to_string()));
        assert_eq!(g_ab.next_node_in_path(&l_b), Some("c".to_string()));
        assert_eq!(g_ab.next_node_in_path(&g_a), Some("b".to_string()));
        assert_eq!(g_ab.next_node_in_path(&g_b), Some("c".to_string()));
        assert_eq!(l_ab.next_node_in_path(&l_a), Some("b".to_string()));
        assert_eq!(l_ab.next_node_in_path(&l_b), Some("c".to_string()));
        assert_eq!(l_ab.next_node_in_path(&g_a), Some("b".to_string()));
        assert_eq!(l_ab.next_node_in_path(&g_b), Some("c".to_string()));
        assert_eq!(l_ab.next_node_in_path(&l_k), None);

    }
}
