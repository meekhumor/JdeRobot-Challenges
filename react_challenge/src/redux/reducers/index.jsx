import { combineReducers } from 'redux';
import taskReducer from './taskReducer';
import categoryReducer from './categoryReducer';
import filterReducer from './filterReducer';
import themeReducer from './themeReducer';

const rootReducer = combineReducers({
  tasks: taskReducer,
  categories: categoryReducer,
  filter: filterReducer,
  theme: themeReducer,
});

export default rootReducer;