import React from "react";
import {
  X,
  Plus,
  Edit,
  Move,
  CheckCircle,
  Trash2,
  Bell,
} from "lucide-react";

const HelpGuide = ({ onClose }) => {
  return (
    <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50">
      <div
        className="bg-white dark:bg-gray-800 rounded-xl shadow-2xl w-full max-w-xl p-8 transform transition-all duration-300 scale-100 hover:scale-105"
      >
        <div className="flex items-center justify-end mb-6">
          <button
            onClick={onClose}
            className="text-gray-500 hover:text-gray-700 dark:text-gray-400 dark:hover:text-gray-200"
          >
            <X size={28} />
          </button>
        </div>

        <div className="max-h-[32rem] overflow-y-auto space-y-6">
          {/* add task */}
          <div className="flex items-start space-x-4 p-4 bg-indigo-50 dark:bg-indigo-900 rounded-lg">
            <Plus className="text-indigo-500 dark:text-indigo-300 flex-shrink-0" size={24} />
            <div>
              <h3 className="font-semibold text-indigo-700 dark:text-indigo-300 text-lg">
                Add Tasks
              </h3>
              <p className="text-base text-gray-700 dark:text-gray-300">
                Enter a title, pick a category, set a priority, and choose a due
                date in the input field below.
              </p>
            </div>
          </div>

          {/* edit task */}
          <div className="flex items-start space-x-4 p-4 bg-green-50 dark:bg-green-900 rounded-lg">
            <Edit className="text-green-500 dark:text-green-300 flex-shrink-0" size={24} />
            <div>
              <h3 className="font-semibold text-green-700 dark:text-green-300 text-lg">
                Edit Tasks
              </h3>
              <p className="text-base text-gray-700 dark:text-gray-300">
                Click the <Edit size={16} className="inline" /> icon, update the
                title, then press Enter to save or Escape to cancel.
              </p>
            </div>
          </div>

          {/* reorder task */}
          <div className="flex items-start space-x-4 p-4 bg-yellow-50 dark:bg-yellow-900 rounded-lg">
            <Move className="text-yellow-500 dark:text-yellow-300 flex-shrink-0" size={24} />
            <div>
              <h3 className="font-semibold text-yellow-700 dark:text-yellow-300 text-lg">
                Reorder Tasks
              </h3>
              <p className="text-base text-gray-700 dark:text-gray-300">
                Drag and drop tasks to prioritize them your way.
              </p>
            </div>
          </div>

          {/* complete task */}
          <div className="flex items-start space-x-4 p-4 bg-blue-50 dark:bg-blue-900 rounded-lg">
            <CheckCircle className="text-blue-500 dark:text-blue-300 flex-shrink-0" size={24} />
            <div>
              <h3 className="font-semibold text-blue-700 dark:text-blue-300 text-lg">
                Complete Tasks
              </h3>
              <p className="text-base text-gray-700 dark:text-gray-300">
                Click the circle next to a task to mark it done.
              </p>
            </div>
          </div>

          {/* delete task */}
          <div className="flex items-start space-x-4 p-4 bg-red-50 dark:bg-red-900 rounded-lg">
            <Trash2 className="text-red-500 dark:text-red-300 flex-shrink-0" size={24} />
            <div>
              <h3 className="font-semibold text-red-700 dark:text-red-300 text-lg">
                Delete Tasks
              </h3>
              <p className="text-base text-gray-700 dark:text-gray-300">
                Hit the <Trash2 size={16} className="inline" /> icon to remove a
                task permanently.
              </p>
            </div>
          </div>

          {/* notifications */}
          <div className="flex items-start space-x-4 p-4 bg-purple-50 dark:bg-purple-900 rounded-lg">
            <Bell className="text-purple-500 dark:text-purple-300 flex-shrink-0" size={24} />
            <div>
              <h3 className="font-semibold text-purple-700 dark:text-purple-300 text-lg">
                Stay Notified
              </h3>
              <p className="text-base text-gray-700 dark:text-gray-300">
                Check the <Bell size={16} className="inline" /> icon for tasks
                due within 3 days.
              </p>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default HelpGuide;