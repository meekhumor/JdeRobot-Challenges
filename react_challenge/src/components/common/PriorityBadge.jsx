import React from 'react';
import { ArrowUp, ArrowRight, ArrowDown } from 'lucide-react';

const priorityConfig = {
  high: { 
    bg: 'bg-red-100', 
    text: 'text-red-800',
    darkBg: 'dark:bg-red-900',
    darkText: 'dark:text-red-200',
    icon: ArrowUp 
  },
  medium: { 
    bg: 'bg-orange-100', 
    text: 'text-orange-800',
    darkBg: 'dark:bg-orange-900',
    darkText: 'dark:text-orange-200',
    icon: ArrowRight 
  },
  low: { 
    bg: 'bg-blue-100', 
    text: 'text-blue-800',
    darkBg: 'dark:bg-blue-900',
    darkText: 'dark:text-blue-200',
    icon: ArrowDown 
  }
};

const PriorityBadge = ({ priority }) => {
  const config = priorityConfig[priority] || priorityConfig.medium;
  const Icon = config.icon;
  
  return (
    <div className={`inline-flex items-center rounded-full px-2 py-1 text-xs ${config.bg} ${config.text} ${config.darkBg} ${config.darkText}`}>
      <Icon size={12} className="mr-1" />
      {priority}
    </div>
  );
};

export default PriorityBadge;