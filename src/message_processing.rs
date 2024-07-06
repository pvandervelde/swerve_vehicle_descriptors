use std::{
    collections::HashMap,
    fmt::Display,
    sync::{
        atomic::{AtomicUsize, Ordering},
        Arc, Mutex,
    },
    thread::{self, JoinHandle},
    time::Duration,
};

use crossbeam_channel::{Receiver, Sender};

use crate::Error;

#[cfg(test)]
#[path = "message_processing_tests.rs"]
mod message_processing_tests;

/// The TaskID counter value for the 'NONE' ID.
static NONE_TASK_ID: usize = 0;

/// Atomic counter for TaskID instances
/// The counter starts at 1 because 0 is reserved for the 'NONE' ID.
static TASK_ID_COUNTER: AtomicUsize = AtomicUsize::new(1);

/// Defines a unique ID for task types
///
/// - Can be cloned safely
/// - Can be created safely across many threads
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct TaskID {
    /// The internal value that forms the actual ID. This is set in a
    /// thread-safe maner
    /// Based on this StackOverflow answer: https://stackoverflow.com/a/32936288/539846
    id: usize,
}

impl TaskID {
    /// Create a reference for the current ID.
    pub fn as_ref(&self) -> &Self {
        &self
    }

    /// Returns a value indicating if the given ID is the [none] ID.
    pub fn is_none(&self) -> bool {
        self.id == NONE_TASK_ID
    }

    /// Create a new ID in a thread safe manner.
    pub fn new() -> Self {
        Self {
            id: TASK_ID_COUNTER.fetch_add(1, Ordering::SeqCst),
        }
    }

    /// Returns the TaskID that doesn't belong to any FrameElement. Can be used to initialize
    /// IDs that are unknown.
    pub fn none() -> Self {
        Self { id: NONE_TASK_ID }
    }
}

impl Display for TaskID {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "TaskID [{}]", self.id)
    }
}

/// An inner struct that stores the state of the task scheduler queue
struct TaskSchedulerQueueState {
    /// The map of functions that the task scheduler will run when a notification
    /// of change comes through.
    ready_queue: HashMap<TaskID, Box<dyn Fn() + Sync + Send>>,

    /// A flag indicating whether or not the task scheduler jobs are being cancelled.
    cancelled: bool,
}

impl TaskSchedulerQueueState {
    /// Creates a new instance of the TaskScheduleQueueState structure.
    fn new() -> Self {
        Self {
            ready_queue: HashMap::new(),
            cancelled: false,
        }
    }
}

/// Defines a scheduler that waits for updates to tasks and executes a closure when it
/// gets a notification of an update.
pub struct TaskScheduler {
    /// The template of the channel sender that is used to notify the scheduler when
    /// there is an update for one of the tasks
    sender_template: Sender<TaskID>,

    /// The thread handle for the background update thread
    background_runner: JoinHandle<()>,

    /// The queue containing the tasks that the background thread runs through
    queue: Arc<Mutex<TaskSchedulerQueueState>>,
}

impl TaskScheduler {
    /// Adds a new task to the scheduler and returns the [TaskID] that is used to notify the
    /// scheduler that the task has an update waiting.
    ///
    /// ## Parameters
    ///
    /// 'closure' - The task that should be executed.
    pub fn add(
        &self,
        closure: Box<dyn Fn() + Sync + Send>,
    ) -> Result<(Sender<TaskID>, TaskID), Error> {
        let result = TaskID::new();
        {
            let guard = self.queue.lock();

            let mut map = guard.unwrap_or_else(|err| err.into_inner());
            map.ready_queue.insert(result.clone(), closure);
        }

        Ok((self.sender_template.clone(), result))
    }

    /// Creates the background task update thread
    fn create_thread<F: FnOnce() + Send + 'static>(f: F) -> JoinHandle<()> {
        thread::spawn(f)
    }

    /// Creates a new [TaskScheduler] instance
    ///
    /// This creates a new background thread that waits for [TaskID]s to be received. Once a
    /// [TaskID] is received
    ///
    /// ## Parameters
    ///
    /// * 'processing_rate_in_hz' - The rate at which tasks should be processed.
    pub fn new(processing_rate_in_hz: i32) -> Self {
        let (s, r) = crossbeam_channel::unbounded();

        let queue = Arc::new(Mutex::new(TaskSchedulerQueueState::new()));
        let queue_copy = queue.clone();

        let background_runner = Self::create_thread(move || {
            let internal_queue = &queue_copy;
            let receiver = &r;
            Self::run(internal_queue, receiver, processing_rate_in_hz);
        });

        Self {
            sender_template: s,
            background_runner,
            queue,
        }
    }

    /// Runs the task processing.
    fn run(
        queue: &Arc<Mutex<TaskSchedulerQueueState>>,
        receiver: &Receiver<TaskID>,
        rate_in_hz: i32,
    ) {
        let sleep_time_in_millis = ((1.0 / (rate_in_hz as f64)) * 1000.0) as u64;
        loop {
            let is_cancelled: bool;
            {
                let arc_lock = queue.lock().unwrap_or_else(|err| err.into_inner());
                is_cancelled = arc_lock.cancelled;
            }

            if is_cancelled {
                break;
            }

            // check the receiver
            let result = receiver.try_recv();
            if result.is_ok() {
                let id = result.unwrap();

                // unwrap the hashmap and see if we have the ID
                let func: Option<&Box<dyn Fn() + Sync + Send>>;
                {
                    let map = queue.lock().unwrap_or_else(|err| err.into_inner());
                    func = map.ready_queue.get(&id);

                    match func {
                        Some(f) => {
                            f();
                        }
                        None => {
                            // The ID didn't exist in our map, but we did have an ID, so we just continue
                            // and go around the loop again to see if there's another ID waiting
                        }
                    };
                }
            } else {
                // There was nothing in the channel, so we wait our normal wait time.
                // This is ugly and there should be a better way of doing this ... Maybe async?
                //
                // In order to do this right we should really count how many milliseconds have past since the
                // last time we slept(??) and then set our duration - wake time (give or take)
                thread::sleep(Duration::from_millis(sleep_time_in_millis));
            }
        }

        // Exit because we're done
    }
}

impl Drop for TaskScheduler {
    fn drop(&mut self) {
        {
            let mut arc_lock = self.queue.lock().unwrap_or_else(|err| err.into_inner());
            arc_lock.cancelled = true;
        }
    }
}
