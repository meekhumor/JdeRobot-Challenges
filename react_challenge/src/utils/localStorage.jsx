export const loadState = () => {
    try {
      const serializedState = localStorage.getItem('taskManagerState');
      if (serializedState === null) {
        return undefined; // Let reducers use their initial state
      }
      return JSON.parse(serializedState);
    } catch (err) {
      console.error('Could not load state:', err);
      return undefined;
    }
  };
  
  export const saveState = (state) => {
    try {
      const serializedState = JSON.stringify(state);
      localStorage.setItem('taskManagerState', serializedState);
    } catch (err) {
      console.error('Could not save state:', err);
    }
  };
  
  // Middleware-like function to debounce saving
  export const debounceSave = (state, delay = 1000) => {
    let timer;
    return () => {
      clearTimeout(timer);
      timer = setTimeout(() => saveState(state), delay);
    };
  };