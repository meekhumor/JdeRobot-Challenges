import React from 'react';

const CardSkeleton = () => {
  return (
    <div className="bg-white dark:bg-gray-800 rounded-lg shadow-md p-4 mb-4 border-l-4 border-gray-300 dark:border-gray-600 animate-pulse">
      <div className="flex items-start justify-between">
        <div className="flex items-start space-x-3 flex-grow">
          {/* Checkbox Placeholder */}
          <div className="w-5 h-5 bg-gray-200 dark:bg-gray-700 rounded-full mt-1" />
          
          {/* Content Placeholder */}
          <div className="flex-grow">
            <div className="h-4 bg-gray-200 dark:bg-gray-700 rounded w-3/4 mb-2" />
            <div className="space-y-2">
              <div className="h-3 bg-gray-200 dark:bg-gray-700 rounded w-1/4" />
              <div className="h-3 bg-gray-200 dark:bg-gray-700 rounded w-1/3" />
            </div>
          </div>
        </div>
        
        {/* Actions Placeholder */}
        <div className="flex space-x-2 ml-2">
          <div className="w-4 h-4 bg-gray-200 dark:bg-gray-700 rounded" />
          <div className="w-4 h-4 bg-gray-200 dark:bg-gray-700 rounded" />
        </div>
      </div>
    </div>
  );
};

export default CardSkeleton;