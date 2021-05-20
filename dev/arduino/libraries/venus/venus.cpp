#include "Arduino.h"
#include "venus.h"
#include "Vector.h"

State::State(StateMachine* state_machine) {
    state_machine = state_machine;
    enter_tasks.setStorage(enter_task_storage_array);
    idle_tasks.setStorage(idle_task_storage_array);
    leave_tasks.setStorage(leave_task_storage_array);
}

void State::add_tasks(Task* task_arr[], int task_count, int type) {
    if (type == 0) {
        for (int i = 0; i < task_count; i++) {
            enter_tasks.push_back(task_arr[i]);
        }
    }
    if (type == 1) {
        for (int i = 0; i < task_count; i++) {
            idle_tasks.push_back(task_arr[i]);
        }
    }
    if (type == 2) {
        for (int i = 0; i < task_count; i++) {
            leave_tasks.push_back(task_arr[i]);
        }
    }
}

void State::update_state(StateMachine* state_machine) {
    switch(task_mode) {
        case enter:
            for (auto task = enter_tasks.begin(); task != enter_tasks.end(); task++) {
                if (state_machine->run_time - (*task)->task_time >= (*task)->task_delta) {
                    (*task)->perform(state_machine);
                }
            }
            task_mode = idle;
            break;
        case idle:
            for (auto task = idle_tasks.begin(); task != idle_tasks.end(); task++) {
                if (state_machine->run_time - (*task)->task_time >= (*task)->task_delta) {
                    (*task)->perform(state_machine);
                }
            }
            break;
        case leave:
            for (auto task = leave_tasks.begin(); task != leave_tasks.end(); task++) {
                if (state_machine->run_time - (*task)->task_time >= (*task)->task_delta) {
                    (*task)->perform(state_machine);
                }
            }
            task_mode = enter;
            state_machine->current_state = next_state;
            break;
    }
}

void StateMachine::add_states(State* state_arr[], int state_count) {
    for (int i = 0; i < state_count; i++) {
        states.push_back(state_arr[i]);
    }
}

StateMachine::StateMachine(float run_time, int start_state) {
    states.setStorage(state_storage_array);
    current_state = start_state;
    run_time = run_time;
}

void StateMachine::states_update() {
    states[current_state]->update_state(this);
}

Task::Task(void (*task_func_in)(StateMachine* state_machine), float task_delta_in) {
    task_delta = task_delta_in;
    task_func = task_func_in;
}

void Task::perform(StateMachine* state_machine) {
    old_task_time = task_time;
    task_time = state_machine->run_time;
    if ((task_time - old_task_time) > task_delta) {
        Serial.println("took too long");
    }
    task_func(state_machine);
}