import { ADD_TASK, TOGGLE_TASK, REMOVE_TASK, UPDATE_TASK, REORDER_TASKS } from '../actions/taskActions';

const initialState = [];

const taskReducer = (state = initialState, action) => {
  switch (action.type) {
    case ADD_TASK:
      return [...state, action.payload];
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
    case REORDER_TASKS: {
      const { dragIndex, hoverIndex } = action.payload;
      const newTasks = [...state];
      const [draggedTask] = newTasks.splice(dragIndex, 1);
      newTasks.splice(hoverIndex, 0, draggedTask);
      return newTasks;
    }
    default:
      return state;
  }
};

export default taskReducer;