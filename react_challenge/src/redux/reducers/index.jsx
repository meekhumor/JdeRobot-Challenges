import { combineReducers } from 'redux';
import taskReducer from './taskReducer';
import categoryReducer from './categoryReducer';
import filterReducer from './filterReducer';

const rootReducer = combineReducers({
  tasks: taskReducer,
  categories: categoryReducer,
  filter: filterReducer,
});

export default rootReducer;