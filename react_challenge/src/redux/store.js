import { createStore, applyMiddleware } from 'redux';
import { thunk } from 'redux-thunk'; 
import rootReducer from './reducers';
import { loadState, saveState, debounce } from '../components/common/LocalStorage';

const persistedState = loadState();
const store = createStore(rootReducer, persistedState, applyMiddleware(thunk));

store.subscribe(debounce(() => {
  saveState({
    tasks: store.getState().tasks,
    categories: store.getState().categories,
    theme: store.getState().theme,
  });
}, 1000)); 

export default store;