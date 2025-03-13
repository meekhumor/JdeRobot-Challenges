import React from 'react';
import { createRoot } from 'react-dom/client';
import { Provider } from 'react-redux'; // Add this import
import store from './redux/store'; // Import your Redux store
import App from './components/App';
import './styles/index.css';

const container = document.getElementById('root');
const root = createRoot(container);

root.render(
  <Provider store={store}>
    <App />
  </Provider>
);