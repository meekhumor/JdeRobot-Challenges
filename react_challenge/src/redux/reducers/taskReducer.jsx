import { ADD_TASK, TOGGLE_TASK, REMOVE_TASK, UPDATE_TASK, REORDER_TASKS } from '../actions/taskActions';

const initialState = [
  // Example initial tasks with order
  { id: 1, title: 'Task 1', completed: false, priority: 'high', dueDate: '2025-03-14', order: 0 },
  { id: 2, title: 'Task 2', completed: false, priority: 'low', dueDate: '2025-03-15', order: 1 },
];

const taskReducer = (state = initialState, action) => {
  switch (action.type) {
    case ADD_TASK:
      return [...state, { ...action.payload, order: state.length }];
    case TOGGLE_TASK:
      return state.map(task =>
        task.id === action.payload ? { ...task, completed: !task.completed } : task
      );
    case REMOVE_TASK:
      return state.filter(task => task.id !== action.payload);
    case UPDATE_TASK:
      return state.map(task =>
        task.id === action.payload.id ? { ...task, ...action.payload.updates } : task
      );
    case REORDER_TASKS:
      return action.payload;
    default:
      return state;
  }
};

export default taskReducer;