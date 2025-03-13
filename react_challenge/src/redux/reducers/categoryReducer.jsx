import { ADD_CATEGORY, REMOVE_CATEGORY } from '../actions/categoryActions';

const initialState = [
  { id: 1, name: 'personal' },
  { id: 2, name: 'work' },
  { id: 3, name: 'groceries' },
  { id: 4, name: 'health' },
  { id: 5, name: 'finance' },
  { id: 6, name: 'education' },
];

const categoryReducer = (state = initialState, action) => {
  switch (action.type) {
    case ADD_CATEGORY:
      return [...state, action.payload];
    case REMOVE_CATEGORY:
      return state.filter(category => category.id !== action.payload);
    default:
      return state;
  }
};

export default categoryReducer;