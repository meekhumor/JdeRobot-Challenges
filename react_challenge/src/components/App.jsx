import React from 'react';
import { DndProvider } from 'react-dnd';
import { HTML5Backend } from 'react-dnd-html5-backend';

import Layout from './layout/Layout';
import TaskForm from './tasks/TaskForm';
import TaskFilter from './tasks/TaskFilter';
import TaskList from './tasks/TaskList';
import SearchBar from './common/SearchBar';

const App = () => {
  return (
    <DndProvider backend={HTML5Backend}>
      <Layout>
        <div className="grid grid-cols-1 lg:grid-cols-4 gap-6">

          <div className="lg:col-span-1">
            <div className="space-y-6">
              <SearchBar />
              <TaskFilter />
            </div>
          </div>
          
          <div className="lg:col-span-3">
            <div className="space-y-6">
              <TaskForm />
              <TaskList />
            </div>
          </div>

        </div>
      </Layout>
    </DndProvider>
  );
};

export default App;