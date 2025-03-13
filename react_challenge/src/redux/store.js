import { createStore, applyMiddleware } from 'redux';
import {thunk} from 'redux-thunk'; // Correct import syntax
import rootReducer from './reducers';
import { loadState, saveState, debounce } from '../components/common/LocalStorage';

const persistedState = loadState();
const store = createStore(rootReducer, persistedState, applyMiddleware(thunk));

store.subscribe(debounce(() => {
  saveState({
    tasks: store.getState().tasks,
    categories: store.getState().categories,
  });
}, 1000));

export default store;