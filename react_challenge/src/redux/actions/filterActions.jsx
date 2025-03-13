export const SET_FILTER = 'SET_FILTER';
export const SET_CATEGORY_FILTER = 'SET_CATEGORY_FILTER';
export const SET_PRIORITY_FILTER = 'SET_PRIORITY_FILTER';
export const SET_SEARCH_QUERY = 'SET_SEARCH_QUERY';

export const setFilter = (status) => ({
  type: SET_FILTER,
  payload: status,
});

export const setCategoryFilter = (category) => ({
  type: SET_CATEGORY_FILTER,
  payload: category,
});

export const setPriorityFilter = (priority) => ({
  type: SET_PRIORITY_FILTER,
  payload: priority,
});

export const setSearchQuery = (query) => ({
  type: SET_SEARCH_QUERY,
  payload: query,
});