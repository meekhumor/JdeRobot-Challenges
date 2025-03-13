import { ADD_CATEGORY, REMOVE_CATEGORY } from '../actions/categoryActions';

const initialState = [
  { id: 1, name: 'default' },
  { id: 2, name: 'personal' },
  { id: 3, name: 'work' },
  { id: 4, name: 'groceries' },
  { id: 5, name: 'health' },
  { id: 6, name: 'finance' },
  { id: 7, name: 'education' },
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