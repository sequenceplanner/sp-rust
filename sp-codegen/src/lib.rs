//! # Code generation for SP

pub mod ros2_common;
pub use crate::ros2_common::{Directories};

pub mod ros2_python_common;
pub use crate::ros2_python_common::{ConfigurationFile, ReadmeFile, ResourceFile, DescriptionFile, 
    SetupFile, TestCopyrightFile, TestPep257File, TestFlake8File};

pub mod ros2_python_model_based;
pub use crate::ros2_python_model_based::{BasicInterfacerNode, BasicEmulatorNode};

pub mod ros2_messages_common;
pub use crate::ros2_messages_common::{CMakeListsFile};

pub mod utils;
pub use crate::utils::{GetSPModelVariables};

// pub mod ros2_messages;
// pub use crate::ros2_python::{Directories, ConfigurationFile};