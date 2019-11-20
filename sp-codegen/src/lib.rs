//! # Code generation for SP

pub mod ros2_common;
pub use crate::ros2_common::{Directories};

pub mod ros2_python_common;
pub use crate::ros2_python_common::{PyConfigurationFile, PyInitFile, PyReadmeFile, PyResourceFile, 
    PyDescriptionFile, PySetupFile, PyTestCopyrightFile, PyTestPep257File, PyTestFlake8File};

pub mod ros2_python_model_based;
pub use crate::ros2_python_model_based::{PyBasicInterfacerNode, PyBasicEmulatorNode};

pub mod ros2_messages_common;
pub use crate::ros2_messages_common::{MsgCMakeListsFile, MsgDescriptionFile, MsgReadmeFile};

pub mod ros2_messages_model_based;
pub use crate::ros2_messages_model_based::{MsgCMakeListsFile, MsgDescriptionFile};

pub mod utils;
pub use crate::utils::{GetSPModelVariables};