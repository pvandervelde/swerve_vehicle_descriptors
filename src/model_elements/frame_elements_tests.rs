use std::time::Duration;

use crossbeam_channel::Receiver;

use crate::{
    change_notification_processing::ChangeID, model_elements::frame_elements::*,
    number_space::NumberSpaceType,
};

// FrameID tests

#[test]
fn when_creating_new_ids_should_be_unique() {
    // Create a set of IDs in multiple threads and make sure they are not identical

    let count = 10;

    // Arrange
    let mut ids = Vec::with_capacity(count);
    for _ in 0..count {
        ids.push(FrameID::new());
    }

    // Assert
    for i in 0..count - 1 {
        let id = ids[i].as_ref();
        for j in i + 1..count {
            let other_id = ids[j].as_ref();
            assert_ne!(id, other_id);
        }
    }
}

#[test]
fn when_creating_new_ids_should_never_match_the_none_id() {
    let count = 10;

    // Arrange
    let mut ids = Vec::with_capacity(count);
    for _ in 0..count {
        ids.push(FrameID::new());
    }

    // Assert
    let none = FrameID::none();
    assert!(none.is_none());

    for i in 0..count - 1 {
        let id = ids[i].as_ref();
        assert_ne!(id, &none);
        assert!(!id.is_none());
    }
}

#[test]
fn when_comparing_id_with_itself_should_be_equal() {
    let id = FrameID::new();
    let copy = id.clone();

    assert_eq!(id, copy)
}

// ReferenceFrame tests

#[test]
fn when_creating_reference_frame_should_be_initialized() {
    let name = "a".to_string();
    let degree_of_freedom_kind = FrameDofType::PrismaticX;
    let is_actuated = true;

    let element = ReferenceFrame::new(name.clone(), degree_of_freedom_kind, is_actuated);

    assert_eq!(name, element.name());
    assert_ne!(&FrameID::none(), element.id());
    assert_eq!(degree_of_freedom_kind, element.degree_of_freedom_kind());
    assert_eq!(is_actuated, element.is_actuated());
}

// ChassisElement

#[test]
fn when_creating_chassis_element_should_be_initialized() {
    let name = "a".to_string();
    let mass = 10.0;
    let center_of_mass = Vector3::new(1.0, 2.0, 3.0);

    #[rustfmt::skip]
    let moment_of_inertia = Matrix3::new(
        11.0, 12.0, 13.0,
        21.0, 22.0, 23.0,
        31.0, 32.0, 33.0);

    #[rustfmt::skip]
    let spatial_inertia = Matrix6::new(
        11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        21.0, 22.0, 23.0, 24.0, 25.0, 26.0,
        31.0, 32.0, 33.0, 34.0, 35.0, 36.0,
        41.0, 42.0, 43.0, 44.0, 45.0, 46.0,
        51.0, 52.0, 53.0, 54.0, 55.0, 56.0,
        61.0, 62.0, 63.0, 64.0, 65.0, 66.0);

    let reference_frame = FrameID::new();

    let element = ChassisElement::new(
        name.clone(),
        mass,
        center_of_mass,
        moment_of_inertia,
        spatial_inertia,
        reference_frame,
    );

    assert_eq!(name, element.name());
    assert_eq!(mass, element.mass_in_kg());
    assert_eq!(&center_of_mass, element.center_of_mass());
    assert_eq!(&moment_of_inertia, element.moment_of_inertia());
    assert_eq!(&spatial_inertia, element.spatial_inertia());
    assert_eq!(&reference_frame, element.reference_frame());
}

// JointSensor

struct MockHardwareSensor {
    receiver: Receiver<JointState>,
    sender: Sender<JointState>,
    update_sender: Option<Sender<ChangeID>>,
    id: Option<ChangeID>,
}

impl HardwareSensor for MockHardwareSensor {
    fn get_joint_motion_type(&self) -> NumberSpaceType {
        NumberSpaceType::LinearUnlimited
    }

    fn get_current_state_receiver(&self) -> Result<Receiver<JointState>, Error> {
        Ok(self.receiver.clone())
    }

    fn on_change(&mut self, id: ChangeID, sender: Sender<ChangeID>) {
        self.id = Some(id);
        self.update_sender = Some(sender);
    }

    fn get_joint_range(&self) -> crate::hardware::joint_state::JointStateRange {
        todo!()
    }
}

struct MockHardwareActuator {
    receiver: Receiver<(JointState, ActuatorAvailableRatesOfChange)>,
    sender: Sender<(JointState, ActuatorAvailableRatesOfChange)>,
    command_sender: Sender<JointState>,
    update_sender: Option<Sender<ChangeID>>,
    id: Option<ChangeID>,
}

impl HardwareActuator for MockHardwareActuator {
    fn get_actuator_motion_type(&self) -> NumberSpaceType {
        NumberSpaceType::LinearUnlimited
    }

    fn get_current_state_receiver(
        &self,
    ) -> Result<Receiver<(JointState, ActuatorAvailableRatesOfChange)>, Error> {
        Ok(self.receiver.clone())
    }

    fn get_command_sender(&self) -> Result<Sender<JointState>, Error> {
        Ok(self.command_sender.clone())
    }

    fn on_change(&mut self, id: ChangeID, sender: Sender<ChangeID>) {
        self.id = Some(id);
        self.update_sender = Some(sender);
    }

    fn get_actuator_range(&self) -> crate::hardware::joint_state::JointStateRange {
        todo!()
    }
}

#[test]
fn test_joint_sensor_new() {
    let (sender, receiver) = crossbeam_channel::unbounded();
    let mut hardware_sensor = MockHardwareSensor {
        receiver,
        sender,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(10));

    let result = JointSensor::new(&mut hardware_sensor, &change_processor);
    assert!(result.is_ok());
}

#[test]
fn test_joint_sensor_get_value() {
    let (sender, receiver) = crossbeam_channel::unbounded();
    let mut hardware_sensor = MockHardwareSensor {
        receiver,
        sender,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(1000));

    let sensor = JointSensor::new(&mut hardware_sensor, &change_processor).unwrap();
    let state = JointState::new(1.0, Some(1.0), Some(1.0), Some(1.0));
    hardware_sensor.sender.send(state.clone()).unwrap();
    hardware_sensor
        .update_sender
        .unwrap()
        .send(hardware_sensor.id.unwrap())
        .unwrap();

    // Allow some time to ensure the task is not processed
    std::thread::sleep(Duration::from_millis(20));

    let result = sensor.get_value();
    assert!(result.is_ok());
    assert_eq!(result.unwrap(), state);
}

#[test]
fn test_actuator_new() {
    let (sender, receiver) = crossbeam_channel::unbounded();
    let (cmd_sender, _cmd_receiver) = crossbeam_channel::unbounded();
    let mut hardware_actuator = MockHardwareActuator {
        receiver,
        sender,
        command_sender: cmd_sender,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(10));

    let result = Actuator::new(&mut hardware_actuator, &change_processor);
    assert!(result.is_ok());
}

#[test]
fn test_actuator_get_value() {
    let (sender, receiver) = crossbeam_channel::unbounded();
    let (cmd_sender, _cmd_receiver) = crossbeam_channel::unbounded();
    let mut hardware_actuator = MockHardwareActuator {
        receiver,
        sender,
        command_sender: cmd_sender,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(1000));

    let actuator = Actuator::new(&mut hardware_actuator, &change_processor).unwrap();
    let state = JointState::new(1.0, Some(1.0), Some(1.0), Some(1.0));
    let rates_of_change = ActuatorAvailableRatesOfChange::new(1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
    hardware_actuator
        .sender
        .send((state.clone(), rates_of_change))
        .unwrap();
    hardware_actuator
        .update_sender
        .unwrap()
        .send(hardware_actuator.id.unwrap())
        .unwrap();

    // Allow some time to ensure the task is not processed
    std::thread::sleep(Duration::from_millis(20));

    let result = actuator.get_value();
    assert!(result.is_ok());
    assert_eq!(result.unwrap(), state);
}

#[test]
fn test_actuator_set_value() {
    let (sender, receiver) = crossbeam_channel::unbounded();
    let (cmd_sender, cmd_receiver) = crossbeam_channel::unbounded();
    let mut actuator = MockHardwareActuator {
        receiver,
        sender,
        command_sender: cmd_sender,
        update_sender: None,
        id: None,
    };
    let change_processor = Box::new(HardwareChangeProcessor::new(10));

    let actuator_instance = Actuator::new(&mut actuator, &change_processor).unwrap();
    let state = JointState::new(2.0, Some(2.0), Some(2.0), Some(2.0));
    let result = actuator_instance.set_value(state.clone());
    assert!(result.is_ok());

    let cmd_result = cmd_receiver.recv();
    assert!(cmd_result.is_ok());
    assert_eq!(cmd_result.unwrap(), state);
}
