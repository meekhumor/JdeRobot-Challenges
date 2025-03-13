export const ADD_TASK = 'ADD_TASK';
export const TOGGLE_TASK = 'TOGGLE_TASK';
export const REMOVE_TASK = 'REMOVE_TASK';
export const UPDATE_TASK = 'UPDATE_TASK';
export const REORDER_TASKS = 'REORDER_TASKS';

export const addTask = (task) => ({
  type: ADD_TASK,
  payload: {
    id: Date.now(), // Simple unique ID generation
    completed: false,
    ...task,
  },
});

export const toggleTask = (id) => ({
  type: TOGGLE_TASK,
  payload: id,
});

export const removeTask = (id) => ({
  type: REMOVE_TASK,
  payload: id,
});

export const updateTask = (id, updates) => ({
  type: UPDATE_TASK,
  payload: { id, updates },
});

export const reorderTasks = (dragIndex, hoverIndex) => ({
  type: REORDER_TASKS,
  payload: { dragIndex, hoverIndex },
});