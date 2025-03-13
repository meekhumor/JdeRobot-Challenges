import React, { useCallback } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import { reorderTasks } from '../../redux/actions/taskActions';
import TaskCard from './TaskCard';
import EmptyState from '../common/EmptyState';

const TaskList = () => {
  const dispatch = useDispatch();
  const tasks = useSelector(state => state.tasks);
  const filters = useSelector(state => state.filter);
  
  const filteredTasks = tasks.filter(task => {
    if (filters.status === 'completed' && !task.completed) return false;
    if (filters.status === 'active' && task.completed) return false;
    if (filters.category !== 'all' && task.category !== filters.category) return false;
    if (filters.priority !== 'all' && task.priority !== filters.priority) return false;
    if (filters.searchQuery && !task.title.toLowerCase().includes(filters.searchQuery.toLowerCase())) return false;
    return true;
  });
  
  const priorityOrder = { high: 1, medium: 2, low: 3 };
  const sortedTasks = [...filteredTasks].sort((a, b) => {
    if (a.order !== b.order) return a.order - b.order;
    if (a.completed !== b.completed) return a.completed ? 1 : -1;
    if (a.priority !== b.priority) return priorityOrder[a.priority] - priorityOrder[b.priority];
    if (a.dueDate && b.dueDate) return new Date(a.dueDate) - new Date(b.dueDate);
    if (a.dueDate && !b.dueDate) return -1;
    if (!a.dueDate && b.dueDate) return 1;
    return 0;
  });
  
  const moveTask = useCallback((dragId, hoverId) => {
    dispatch(reorderTasks(dragId, hoverId));
  }, [dispatch]);
  
  if (sortedTasks.length === 0) {
    return <EmptyState type={filters.status} searchQuery={filters.searchQuery} />;
  }
  
  return (
    <div className="space-y-4">
      {sortedTasks.map((task, index) => (
        <TaskCard 
          key={task.id} 
          task={task} 
          index={index} 
          moveTask={moveTask} 
        />
      ))}
    </div>
  );
};

export default TaskList;