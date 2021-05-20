#ifndef venus_h
#define venus_h

#include "Vector.h"
class State;

const int MAX_STATES = 10;
enum task_type {enter, idle, leave};

class StateMachine {
    public:
        State* state_storage_array[MAX_STATES];
        Vector<State*> states;
        int current_state;
        float run_time;

        StateMachine(float run_time, int start_state);
        void states_update();
        void add_states(State* state_arr[], int state_count);
};


class Task {
    public:
        float old_task_time = 0.0;
        float task_time = 0.0;
        float task_delta;
        void (*task_func)(StateMachine*);

        Task(void (*task_func)(StateMachine* state_machine), float task_delta);
        void perform(StateMachine* state_machine);
};

const int MAX_TASKS = 10;

class State {
    public:
        Task* enter_task_storage_array[MAX_TASKS];
        Task* idle_task_storage_array[MAX_TASKS];
        Task* leave_task_storage_array[MAX_TASKS];
        Vector<Task*> enter_tasks;
        Vector<Task*> idle_tasks;
        Vector<Task*> leave_tasks;

        int task_mode = enter;
        int next_state;

        StateMachine* state_machine;

        State(StateMachine* state_machine);
        void update_state(StateMachine* state_machine);
        void State::add_tasks(Task* task_arr[], int task_count, int task_type);
};

#endif