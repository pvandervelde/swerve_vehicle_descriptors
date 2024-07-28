use super::*;
use std::sync::atomic::AtomicBool;
use std::sync::Arc;
use std::time::Duration;

// ChangeID

#[test]
fn when_creating_new_ids_should_be_unique() {
    // Create a set of IDs in multiple threads and make sure they are not identical

    let count = 10;

    // Arrange
    let mut ids = Vec::with_capacity(count);
    for _ in 0..count {
        ids.push(ChangeID::new());
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
        ids.push(ChangeID::new());
    }

    // Assert
    let none = ChangeID::none();
    assert!(none.is_none());

    for i in 0..count - 1 {
        let id = ids[i].as_ref();
        assert_ne!(id, &none);
        assert!(!id.is_none());
    }
}

#[test]
fn when_comparing_id_with_itself_should_be_equal() {
    let id = ChangeID::new();
    let copy = id.clone();

    assert_eq!(id, copy)
}

#[test]
fn when_displaying_an_id_it_should_write_out_the_internal_id_number() {
    let id = ChangeID::none();
    assert_eq!(format!("{}", id), "ChangeID [0]");
}

// HardwareChangeProcessor

#[test]
fn test_task_addition_and_execution() {
    // Create a new HardwareChangeProcessor
    let processing_rate_in_hz = 10;
    let scheduler = HardwareChangeProcessor::new(processing_rate_in_hz);

    // Create a flag to indicate task execution
    let executed_flag = Arc::new(AtomicBool::new(false));
    let executed_flag_clone = executed_flag.clone();

    // Add a task to the scheduler
    let task = move || {
        executed_flag_clone.store(true, Ordering::SeqCst);
    };

    let (sender, task_id) = scheduler.add(Box::new(task)).unwrap();

    // Notify the scheduler of the new task
    sender.send(task_id).unwrap();

    // Allow some time for the task to be processed
    std::thread::sleep(Duration::from_millis(200));

    // Check if the task was executed
    assert!(executed_flag.load(Ordering::SeqCst));
}

#[test]
fn test_task_execution_with_unregistered_task() {
    // Create a new HardwareChangeProcessor
    let processing_rate_in_hz = 10;
    let scheduler = HardwareChangeProcessor::new(processing_rate_in_hz);

    // Create a flag to indicate task execution
    let executed_flag = Arc::new(AtomicBool::new(false));
    let executed_flag_clone = executed_flag.clone();

    // Add a task to the scheduler
    let task = move || {
        executed_flag_clone.store(true, Ordering::SeqCst);
    };

    let (sender, _) = scheduler.add(Box::new(task)).unwrap();

    // Notify the scheduler of the new task
    let unregistered_task = ChangeID::new();
    sender.send(unregistered_task).unwrap();

    // Allow some time for the task to be processed
    std::thread::sleep(Duration::from_millis(200));

    // Check if the task was executed
    assert!(!executed_flag.load(Ordering::SeqCst));
}

#[test]
fn test_task_cancellation() {
    // Create a new HardwareChangeProcessor
    let processing_rate_in_hz = 1000;
    let scheduler = HardwareChangeProcessor::new(processing_rate_in_hz);

    // Create a flag to indicate task execution
    let executed_flag = Arc::new(AtomicBool::new(false));
    let executed_flag_clone = executed_flag.clone();

    // Add a task to the scheduler
    let task = move || {
        executed_flag_clone.store(true, Ordering::SeqCst);
    };

    let (sender, task_id) = scheduler.add(Box::new(task)).unwrap();

    // Cancel the scheduler
    drop(scheduler);

    // Notify the scheduler of the new task
    // This should not execute the task since the scheduler is cancelled
    let result = sender.send(task_id);
    assert!(result.is_ok());

    // Allow some time to ensure the task is not processed
    std::thread::sleep(Duration::from_millis(200));

    // Check if the task was executed
    assert!(!executed_flag.load(Ordering::SeqCst));
}
