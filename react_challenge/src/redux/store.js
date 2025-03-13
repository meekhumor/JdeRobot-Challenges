import { createStore } from 'redux';
import rootReducer from './reducers';
import { loadState, saveState } from '../utils/localStorage';

const persistedState = loadState();
const store = createStore(rootReducer, persistedState);

// Subscribe to store changes and save to localStorage with debounce
store.subscribe(() => {
  saveState({
    tasks: store.getState().tasks,
    categories: store.getState().categories,
    theme: store.getState().theme,
  });
});

export default store;