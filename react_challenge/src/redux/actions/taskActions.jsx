export const ADD_TASK = 'ADD_TASK';
export const TOGGLE_TASK = 'TOGGLE_TASK';
export const REMOVE_TASK = 'REMOVE_TASK';
export const UPDATE_TASK = 'UPDATE_TASK';
export const REORDER_TASKS = 'REORDER_TASKS';

export const addTask = (task) => ({
  type: ADD_TASK,
  payload: {
    id: Date.now(), 
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

export const reorderTasks = (dragId, hoverId) => (dispatch, getState) => {
  const { tasks } = getState();
  const dragIndex =tasks.findIndex(t => t.id === dragId);
  const hoverIndex =tasks.findIndex(t => t.id === hoverId);
  
  if (dragIndex === -1 || hoverIndex === -1) return;

  const newTasks = [...tasks];
  const [draggedTask] = newTasks.splice(dragIndex, 1);
  newTasks.splice(hoverIndex, 0, draggedTask);
  
  newTasks.forEach((task, index) => {
    task.order = index;
  });

  dispatch({
    type: 'REORDER_TASKS',
    payload: newTasks,
  });
};