import React from 'react';
import { useDispatch, useSelector } from 'react-redux';
import { setFilter, setCategoryFilter, setPriorityFilter } from '../../redux/actions/filterActions';
import { CheckCircle, Circle, ListFilter, Tag, Flag } from 'lucide-react';

const TaskFilter = () => {
  const dispatch = useDispatch();
  const filter = useSelector(state => state.filter);
  const categories = useSelector(state => state.categories);
  
  return (
    <div className="bg-white dark:bg-gray-800 rounded-lg shadow-md p-5">
      <h2 className="text-lg font-semibold mb-4 flex items-center text-gray-800 dark:text-gray-200">
        <ListFilter size={18} className="mr-2" />
        Filter Tasks
      </h2>
      
      <div className="space-y-4">

        {/* status filter  */}
        <div>
          <h3 className="text-sm font-medium mb-2 text-gray-700 dark:text-gray-300 flex items-center">
            <Circle size={16} className="mr-2" />
            Status
          </h3>
          <div className="flex flex-wrap gap-2">
            <button
              onClick={() => dispatch(setFilter('all'))}
              className={`px-3 py-1 text-sm rounded-full transition
                ${filter.status === 'all'
                  ? 'bg-indigo-100 text-indigo-800 dark:bg-indigo-900 dark:text-indigo-200'
                  : 'bg-gray-100 text-gray-600 hover:bg-gray-200 dark:bg-gray-700 dark:text-gray-300 dark:hover:bg-gray-600'}`}
            >
              All
            </button>
            <button
              onClick={() => dispatch(setFilter('active'))}
              className={`px-3 py-1 text-sm rounded-full transition flex items-center
                ${filter.status === 'active'
                  ? 'bg-blue-100 text-blue-800 dark:bg-blue-900 dark:text-blue-200'
                  : 'bg-gray-100 text-gray-600 hover:bg-gray-200 dark:bg-gray-700 dark:text-gray-300 dark:hover:bg-gray-600'}`}
            >
              <Circle size={12} className="mr-1" />
              Active
            </button>
            <button
              onClick={() => dispatch(setFilter('completed'))}
              className={`px-3 py-1 text-sm rounded-full transition flex items-center
                ${filter.status === 'completed'
                  ? 'bg-green-100 text-green-800 dark:bg-green-900 dark:text-green-200'
                  : 'bg-gray-100 text-gray-600 hover:bg-gray-200 dark:bg-gray-700 dark:text-gray-300 dark:hover:bg-gray-600'}`}
            >
              <CheckCircle size={12} className="mr-1" />
              Completed
            </button>
          </div>
        </div>
        
        {/* category filter */}
        <div>
          <h3 className="text-sm font-medium mb-2 text-gray-700 dark:text-gray-300 flex items-center">
            <Tag size={16} className="mr-2" />
            Category
          </h3>
          <div className="flex flex-wrap gap-2">
            <button
              onClick={() => dispatch(setCategoryFilter('all'))}
              className={`px-3 py-1 text-sm rounded-full transition
                ${filter.category === 'all'
                  ? 'bg-indigo-100 text-indigo-800 dark:bg-indigo-900 dark:text-indigo-200'
                  : 'bg-gray-100 text-gray-600 hover:bg-gray-200 dark:bg-gray-700 dark:text-gray-300 dark:hover:bg-gray-600'}`}
            >
              All
            </button>
            {categories.map(cat => (
              <button
                key={cat.id}
                onClick={() => dispatch(setCategoryFilter(cat.name))}
                className={`px-3 py-1 text-sm rounded-full transition
                  ${filter.category === cat.name
                    ? 'bg-purple-100 text-purple-800 dark:bg-purple-900 dark:text-purple-200'
                    : 'bg-gray-100 text-gray-600 hover:bg-gray-200 dark:bg-gray-700 dark:text-gray-300 dark:hover:bg-gray-600'}`}
              >
                {cat.name}
              </button>
            ))}
          </div>
        </div>
        
        {/* priority filter */}
        <div>
          <h3 className="text-sm font-medium mb-2 text-gray-700 dark:text-gray-300 flex items-center">
            <Flag size={16} className="mr-2" />
            Priority
          </h3>
          <div className="flex flex-wrap gap-2">
            <button
              onClick={() => dispatch(setPriorityFilter('all'))}
              className={`px-3 py-1 text-sm rounded-full transition
                ${filter.priority === 'all'
                  ? 'bg-indigo-100 text-indigo-800 dark:bg-indigo-900 dark:text-indigo-200'
                  : 'bg-gray-100 text-gray-600 hover:bg-gray-200 dark:bg-gray-700 dark:text-gray-300 dark:hover:bg-gray-600'}`}
            >
              All
            </button>
            <button
              onClick={() => dispatch(setPriorityFilter('high'))}
              className={`px-3 py-1 text-sm rounded-full transition
                ${filter.priority === 'high'
                  ? 'bg-red-100 text-red-800 dark:bg-red-900 dark:text-red-200'
                  : 'bg-gray-100 text-gray-600 hover:bg-gray-200 dark:bg-gray-700 dark:text-gray-300 dark:hover:bg-gray-600'}`}
            >
              High
            </button>
            <button
              onClick={() => dispatch(setPriorityFilter('medium'))}
              className={`px-3 py-1 text-sm rounded-full transition
                ${filter.priority === 'medium'
                  ? 'bg-orange-100 text-orange-800 dark:bg-orange-900 dark:text-orange-200'
                  : 'bg-gray-100 text-gray-600 hover:bg-gray-200 dark:bg-gray-700 dark:text-gray-300 dark:hover:bg-gray-600'}`}
            >
              Medium
            </button>
            <button
              onClick={() => dispatch(setPriorityFilter('low'))}
              className={`px-3 py-1 text-sm rounded-full transition
                ${filter.priority === 'low'
                  ? 'bg-blue-100 text-blue-800 dark:bg-blue-900 dark:text-blue-200'
                  : 'bg-gray-100 text-gray-600 hover:bg-gray-200 dark:bg-gray-700 dark:text-gray-300 dark:hover:bg-gray-600'}`}
            >
              Low
            </button>
          </div>
        </div>
      </div>
    </div>
  );
};

export default TaskFilter;
