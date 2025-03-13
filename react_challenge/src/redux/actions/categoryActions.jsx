export const ADD_CATEGORY = 'ADD_CATEGORY';
export const REMOVE_CATEGORY = 'REMOVE_CATEGORY';

export const addCategory = (name) => ({
  type: ADD_CATEGORY,
  payload: {
    id: Date.now(),
    name: name.toLowerCase(),
  },
});

export const removeCategory = (id) => ({
  type: REMOVE_CATEGORY,
  payload: id,
});