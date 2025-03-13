import React from 'react';
import { CheckCircle, Circle, Search } from 'lucide-react';

const EmptyState = ({ type, searchQuery }) => {
  let icon = <Circle size={48} className="text-gray-400" />;
  let title = "No tasks yet";
  let message = "Add your first task to get started";
  
  if (searchQuery) {
    icon = <Search size={48} className="text-gray-400" />;
    title = "No matching tasks";
    message = `No tasks found matching "${searchQuery}"`;
  } else if (type === 'completed') {
    icon = <CheckCircle size={48} className="text-gray-400" />;
    title = "No completed tasks";
    message = "Tasks you complete will appear here";
  } else if (type === 'active') {
    icon = <Circle size={48} className="text-gray-400" />;
    title = "No active tasks";
    message = "All your tasks are completed!";
  }
  
  return (
    <div className="flex flex-col items-center justify-center py-16 text-center">
      <div className="mb-4">{icon}</div>
      <h3 className="text-xl font-semibold text-gray-700 dark:text-gray-300 mb-2">{title}</h3>
      <p className="text-gray-500 dark:text-gray-400">{message}</p>
    </div>
  );
};

export default EmptyState