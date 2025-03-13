import React from 'react';
import { Tag } from 'lucide-react';

const categoryColors = {
  default: { bg: 'bg-gray-100', text: 'text-gray-800', darkBg: 'dark:bg-gray-700', darkText: 'dark:text-gray-200' },
  personal: { bg: 'bg-purple-100', text: 'text-purple-800', darkBg: 'dark:bg-purple-900', darkText: 'dark:text-purple-200' },
  work: { bg: 'bg-blue-100', text: 'text-blue-800', darkBg: 'dark:bg-blue-900', darkText: 'dark:text-blue-200' },
  groceries: { bg: 'bg-green-100', text: 'text-green-800', darkBg: 'dark:bg-green-900', darkText: 'dark:text-green-200' },
  health: { bg: 'bg-red-100', text: 'text-red-800', darkBg: 'dark:bg-red-900', darkText: 'dark:text-red-200' },
  finance: { bg: 'bg-yellow-100', text: 'text-yellow-800', darkBg: 'dark:bg-yellow-900', darkText: 'dark:text-yellow-200' },
  education: { bg: 'bg-indigo-100', text: 'text-indigo-800', darkBg: 'dark:bg-indigo-900', darkText: 'dark:text-indigo-200' },
};

const CategoryBadge = ({ category }) => {
  const color = categoryColors[category] || categoryColors.default;
  
  return (
    <div className={`inline-flex items-center rounded-full px-2 py-1 text-xs ${color.bg} ${color.text} ${color.darkBg} ${color.darkText}`}>
      <Tag size={12} className="mr-1" />
      {category}
    </div>
  );
};

export default CategoryBadge;