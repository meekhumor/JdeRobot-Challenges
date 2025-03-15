import React, { useState, useEffect } from "react";
import { useSelector } from "react-redux";
import { Moon, Sun, Bell, HelpCircle } from "lucide-react";
import HelpGuide from "../common/HelpGuide";

const Header = () => {
  const [showNotifications, setShowNotifications] = useState(false);
  const [showHelp, setShowHelp] = useState(false);
  const [darkMode, setDarkMode] = useState(true);

  const tasks = useSelector((state) => state.tasks);

  useEffect(() => {
    if (darkMode) {
      document.documentElement.classList.add("dark");
    } else {
      document.documentElement.classList.remove("dark");
    }
  }, [darkMode]);

  const upcomingTasks = tasks.filter((task) => {
    if (!task.dueDate || task.completed) return false;
    const dueDate = new Date(task.dueDate);
    const today = new Date();
    const diffTime = dueDate - today;
    const diffDays = Math.ceil(diffTime / (1000 * 60 * 60 * 24));
    return diffDays >= 0 && diffDays <= 3;
  });

  const toggleNotifications = () => setShowNotifications(!showNotifications);
  const toggleHelp = () => setShowHelp(!showHelp);

  return (
    <header className="bg-indigo-600 text-white shadow-lg">
      <div className="container mx-auto px-4">
        <div className="flex items-center justify-between py-4">

          {/* logo */}
          <div className="flex items-center space-x-3">
            <div className="h-10 w-10 bg-white rounded-lg flex items-center justify-center">
              <span className="text-indigo-600 font-bold text-xl">TB</span>
            </div>
            <h1 className="text-2xl font-bold hidden md:block">TaskBuddy</h1>
          </div>

          <div className="flex items-center space-x-4">

            {/* notification button */}
            <div className="relative">
              <button
                onClick={toggleNotifications}
                className="p-2 hover:bg-indigo-500 rounded-full transition relative"
              >
                <Bell size={20} />
                {upcomingTasks.length > 0 && (
                  <span className="absolute top-0 right-0 h-4 w-4 bg-red-500 rounded-full text-xs flex items-center justify-center">
                    {upcomingTasks.length}
                  </span>
                )}
              </button>

              {/* notification dropdown */}
              {showNotifications && (
                <div className="absolute right-0 mt-2 w-80 bg-white dark:bg-gray-800 rounded-lg shadow-lg py-2 z-10 text-gray-800 dark:text-gray-200">
                  <h3 className="px-4 py-2 text-sm font-semibold border-b border-gray-200 dark:border-gray-700">
                    Upcoming Tasks
                  </h3>
                  {upcomingTasks.length === 0 ? (
                    <p className="px-4 py-3 text-sm text-gray-600 dark:text-gray-400">
                      No upcoming tasks
                    </p>
                  ) : (
                    <div className="max-h-60 overflow-y-auto">
                      {upcomingTasks.map((task) => (
                        <div
                          key={task.id}
                          className="px-4 py-2 hover:bg-gray-100 dark:hover:bg-gray-700 border-b border-gray-200 dark:border-gray-700 last:border-0"
                        >
                          <div className="flex justify-between items-start">
                            <span className="font-medium text-sm">
                              {task.title}
                            </span>
                            <span className="text-xs bg-yellow-100 dark:bg-yellow-900 text-yellow-800 dark:text-yellow-200 px-2 py-1 rounded">
                              {new Date(task.dueDate).toLocaleDateString()}
                            </span>
                          </div>
                          <div className="flex mt-1">
                            <span
                              className={`text-xs rounded px-2 py-1 ${
                                task.priority === "high"
                                  ? "bg-red-100 text-red-800 dark:bg-red-900 dark:text-red-200"
                                  : task.priority === "medium"
                                  ? "bg-orange-100 text-orange-800 dark:bg-orange-900 dark:text-orange-200"
                                  : "bg-blue-100 text-blue-800 dark:bg-blue-900 dark:text-blue-200"
                              }`}
                            >
                              {task.priority}
                            </span>
                            <span className="text-xs bg-purple-100 dark:bg-purple-900 text-purple-800 dark:text-purple-200 rounded px-2 py-1 ml-2">
                              {task.category}
                            </span>
                          </div>
                        </div>
                      ))}
                    </div>
                  )}
                </div>
              )}
            </div>

            {/* help button */}
            <button
              onClick={toggleHelp}
              className="p-2 hover:bg-indigo-500 rounded-full transition"
            >
              <HelpCircle size={20} />
            </button>

            {/* darkmode */}
            <button
              onClick={() => setDarkMode(!darkMode)}
              className="p-2 hover:bg-indigo-500 rounded-full transition"
            >
              {darkMode ? <Sun size={20} /> : <Moon size={20} />}
            </button>
          </div>
        </div>
      </div>

      {showHelp && <HelpGuide onClose={toggleHelp} />}
    </header>
  );
};

export default Header;