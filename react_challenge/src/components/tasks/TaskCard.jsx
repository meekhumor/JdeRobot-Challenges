import React, { useState, useRef } from 'react';
import { useDispatch } from 'react-redux';
import { useDrag, useDrop } from 'react-dnd';
import { Calendar, CheckCircle, Circle, Clock, Edit, Trash2, AlertTriangle } from 'lucide-react';
import { toggleTask, removeTask, updateTask } from '../../redux/actions/taskActions';
import CategoryBadge from '../common/CategoryBadge';
import PriorityBadge from '../common/PriorityBadge';

const TaskCard = ({ task, index, moveTask }) => {
  const [isEditing, setIsEditing] = useState(false);
  const [editTitle, setEditTitle] = useState(task.title);
  const dispatch = useDispatch();
  const ref = useRef(null);
  
  const isDueSoon = () => {
    if (!task.dueDate) return false;
    const dueDate = new Date(task.dueDate);
    const today = new Date();
    const diffTime = dueDate - today;
    const diffDays = Math.ceil(diffTime/(1000*60*60*24));
    return diffDays >= 0 && diffDays <= 2; 
  };
  
  const isOverdue = () => {
    if (!task.dueDate) return false;
    const dueDate = new Date(task.dueDate);
    const today = new Date();
    return dueDate < today && !task.completed;
  };
  
  const [{ isDragging }, drag] = useDrag({
    type: 'TASK',
    item: { id: task.id, index },
    collect: (monitor) => ({
      isDragging: monitor.isDragging(),
    }),
  });
  
  const [, drop] = useDrop({
    accept: 'TASK',
    hover: (item, monitor) => {
      if (!ref.current || item.id === task.id) {
        return;
      }
      moveTask(item.id, task.id);  
      item.index = index;  
    },
  });

  drag(drop(ref));
  
  const handleToggle = () => {
    dispatch(toggleTask(task.id));
  };
  
  const handleRemove = () => {
    dispatch(removeTask(task.id));
  };
  
  const handleEdit = () => {
    if (isEditing) {
      dispatch(updateTask(task.id, { title: editTitle }));
    }
    setIsEditing(!isEditing);
  };
  
  const formatDate = (dateString) => {
    const options = { month: 'short', day: 'numeric' };
    return new Date(dateString).toLocaleDateString(undefined, options);
  };
  
  return (
    <div 
      ref={ref}
      className={`bg-white dark:bg-gray-800 rounded-lg shadow-md p-4 mb-4 border-l-8 
        ${task.completed ? 'border-green-500' : 
         isOverdue() ? 'border-red-500' : 
         isDueSoon() ? 'border-yellow-500' : 
         'border-indigo-500'}
        ${isDragging ? 'opacity-50' : 'opacity-100'}
        transition-all duration-300 hover:shadow-lg transform hover:-translate-y-1`}
    >
      <div className="flex items-start justify-between">
        <div className="flex items-start space-x-3 flex-grow">
        
          {/* current status*/}
          <button 
            onClick={handleToggle}
            className={`mt-1 flex-shrink-0 text-${task.completed ? 'green' : 'gray'}-500 hover:text-${task.completed ? 'green' : 'gray'}-700`}
          >
            {task.completed ? <CheckCircle size={20} /> : <Circle size={20} />}
          </button>
          

          <div className="flex-grow">
            {isEditing ? (
              <input
                type="text"
                value={editTitle}
                onChange={(e) => setEditTitle(e.target.value)}
                className="w-full p-1 border border-gray-300 dark:border-gray-600 rounded dark:bg-gray-700 dark:text-white"
                autoFocus
                onKeyDown={(e) => {
                  if (e.key === 'Enter') {
                    handleEdit();
                  }
                }}
              />
            ) : (
              <h3 className={`font-medium text-gray-800 dark:text-gray-200 ${task.completed ? 'line-through text-gray-500 dark:text-gray-400' : ''}`}>
                {task.title}
              </h3>
            )}
            
            {/* details badge */}
            <div className="mt-2 flex flex-wrap gap-2">
              <CategoryBadge category={task.category} />
              <PriorityBadge priority={task.priority} />
              
              {task.dueDate && (
                <div className={`flex items-center text-xs px-2 py-1 rounded-full 
                  ${isOverdue() ? 'bg-red-100 text-red-800 dark:bg-red-900 dark:text-red-200' : 
                   isDueSoon() ? 'bg-yellow-100 text-yellow-800 dark:bg-yellow-900 dark:text-yellow-200' : 
                   'bg-blue-100 text-blue-800 dark:bg-blue-900 dark:text-blue-200'}`}
                >
                  {isOverdue() ? (
                    <AlertTriangle size={12} className="mr-1" />
                  ) : isDueSoon() ? (
                    <Clock size={12} className="mr-1" />
                  ) : (
                    <Calendar size={12} className="mr-1" />
                  )}
                  {formatDate(task.dueDate)}
                </div>
              )}
            </div>
          </div>
        </div>
        
        {/* update and delete task  */}
        <div className="flex space-x-2 ml-2">
          <button 
            onClick={handleEdit}
            className="text-gray-400 hover:text-indigo-500 dark:text-gray-500 dark:hover:text-indigo-400"
          >
            <Edit size={16} />
          </button>
          <button 
            onClick={handleRemove}
            className="text-gray-400 hover:text-red-500 dark:text-gray-500 dark:hover:text-red-400"
          >
            <Trash2 size={16} />
          </button>
        </div>
      </div>
    </div>
  );
};

export default TaskCard;