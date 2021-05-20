from state_machine import State, Task


def define_states(state_machine):
    get_sensors_low_rate = Task(state_machine.get_sensors, task_delta=1.0)
    init_sensors_task = Task(state_machine.init_sensors)
    pad_accel_task = Task(state_machine.pad_accel_wait, task_delta=0.02)
    
    boot_up = State(state_machine,
                    enter_tasks=[init_sensors_task])
    ground_idle = State(state_machine,
                        active_tasks=[get_sensors_low_rate])
    self_test = State(state_machine)
    pad_idle = State(state_machine, 
                     active_tasks=[pad_accel_task])
    launch_ready = State(state_machine)
    power_ascent = State(state_machine)
    coast = State(state_machine)
    drogue_wait = State(state_machine)
    drogue_descent = State(state_machine)
    main_descent = State(state_machine)
    landing = State(state_machine)
    post_landing = State(state_machine)
    safe_idle = State(state_machine)
    shutdown = State(state_machine)

    state_machine.state_list = [ground_idle, pad_idle, power_ascent]
    