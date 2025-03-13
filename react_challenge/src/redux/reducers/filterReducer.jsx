import { SET_FILTER, SET_CATEGORY_FILTER, SET_PRIORITY_FILTER, SET_SEARCH_QUERY } from '../actions/filterActions';

const initialState = {
  status: 'all',
  category: 'all',
  priority: 'all',
  searchQuery: '',
};

const filterReducer = (state = initialState, action) => {
  switch (action.type) {
    case SET_FILTER:
      return { ...state, status: action.payload };
    case SET_CATEGORY_FILTER:
      return { ...state, category: action.payload };
    case SET_PRIORITY_FILTER:
      return { ...state, priority: action.payload };
    case SET_SEARCH_QUERY:
      return { ...state, searchQuery: action.payload };
    default:
      return state;
  }
};

export default filterReducer;