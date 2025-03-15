import React, { useState } from "react";
import { useDispatch, useSelector } from "react-redux";
import DatePicker from "react-datepicker";
import "react-datepicker/dist/react-datepicker.css";
import { addTask } from "../../redux/actions/taskActions";
import { addCategory } from "../../redux/actions/categoryActions";
import { Plus, ChevronDown } from "lucide-react";

const TaskForm = () => {
  const [title, setTitle] = useState("");
  const [priority, setPriority] = useState("medium");
  const [category, setCategory] = useState("personal");
  const [dueDate, setDueDate] = useState(null);
  const [showAdvanced, setShowAdvanced] = useState(false);
  const [showNewCategoryInput, setShowNewCategoryInput] = useState(false);
  const [newCategory, setNewCategory] = useState("");

  const dispatch = useDispatch();
  const categories = useSelector((state) => state.categories);

  const handleSubmit = (e) => {
    e.preventDefault();
    if (!title.trim()) return;

    dispatch(
      addTask({
        title: title.trim(),
        priority,
        category,
        dueDate,
      })
    );

    setTitle("");
    setPriority("medium");
    setCategory("personal");
    setDueDate(null);
  };

  const handleAddCategory = () => {
    if (newCategory.trim()) {
      const normalizedCategory = newCategory.toLowerCase();
      dispatch(addCategory(normalizedCategory));
      setCategory(normalizedCategory);
      setNewCategory("");
      setShowNewCategoryInput(false);
    }
  };

  const isSubmitDisabled = !title.trim();

  return (
    <div className="bg-white dark:bg-gray-800 rounded-lg shadow-md p-5">
      <h2 className="text-lg font-semibold mb-4 text-gray-800 dark:text-gray-200">
        Add New Task
      </h2>
      <form onSubmit={handleSubmit}>
        <div className="mb-4">
          <input
            type="text"
            placeholder="What needs to be done?"
            value={title}
            onChange={(e) => setTitle(e.target.value)}
            className="w-full p-2 border border-gray-300 dark:border-gray-600 rounded-md 
                      focus:ring-2 focus:ring-indigo-500 focus:border-indigo-500 transition
                      dark:bg-gray-700 dark:text-white"
          />
        </div>

        <div className="mb-3">
          <button
            type="button"
            onClick={() => setShowAdvanced(!showAdvanced)}
            className="flex items-center text-sm text-gray-600 dark:text-gray-400 hover:text-indigo-600 dark:hover:text-indigo-400 transition"
          >
            <ChevronDown
              size={16}
              className={`mr-1 transition-transform ${
                showAdvanced ? "transform rotate-180" : ""
              }`}
            />
            {showAdvanced ? "Hide" : "Show"} Advanced Options
          </button>
        </div>

        {showAdvanced && (
          <div className="grid grid-cols-1 sm:grid-cols-2 gap-3 mb-4 animate-fade-in">
            <div>
              <label className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">
                Priority
              </label>
              <select
                value={priority}
                onChange={(e) => setPriority(e.target.value)}
                className="w-full p-2 border border-gray-300 dark:border-gray-600 rounded-md
                          dark:bg-gray-700 dark:text-white"
              >
                <option value="high">High</option>
                <option value="medium">Medium</option>
                <option value="low">Low</option>
              </select>
            </div>

            <div>
              <label className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">
                Category
              </label>
              <div className="flex items-center space-x-2">
                <select
                  value={category}
                  onChange={(e) => setCategory(e.target.value)}
                  className="w-full p-2 border border-gray-300 dark:border-gray-600 rounded-md
                            dark:bg-gray-700 dark:text-white"
                >
                  {categories.map((cat) => (
                    <option key={cat.id} value={cat.name}>
                      {cat.name.charAt(0).toUpperCase() + cat.name.slice(1)}
                    </option>
                  ))}
                </select>
                <button
                  type="button"
                  onClick={() => setShowNewCategoryInput(!showNewCategoryInput)}
                  className="p-2 bg-indigo-600 text-white rounded-md hover:bg-indigo-700 transition"
                >
                  <Plus size={16} />
                </button>
              </div>
              {showNewCategoryInput && (
                <div className="mt-2">
                  <input
                    type="text"
                    value={newCategory}
                    onChange={(e) => setNewCategory(e.target.value)}
                    placeholder="New category"
                    className="w-full p-2 rounded-md dark:bg-gray-700 dark:text-white"
                    onKeyDown={(e) => e.key === "Enter" && handleAddCategory()}
                  />
                  <button
                    type="button"
                    onClick={handleAddCategory}
                    className="mt-1 w-full p-1 bg-indigo-600 text-white rounded-md hover:bg-indigo-700"
                  >
                    Add Category
                  </button>
                </div>
              )}
            </div>

            <div className="sm:col-span-2">
              <label className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">
                Due Date
              </label>
              <DatePicker
                selected={dueDate}
                onChange={(date) => setDueDate(date)}
                placeholderText="Select a due date"
                dateFormat="MMMM d, yyyy"
                minDate={new Date()}
                className="w-full p-2 border border-gray-300 dark:border-gray-600 rounded-md
                          dark:bg-gray-700 dark:text-white"
              />
            </div>
          </div>
        )}

        <button
          type="submit"
          disabled={isSubmitDisabled}
          className={`w-full flex items-center justify-center p-2 rounded-md text-white transition duration-200 ${
            isSubmitDisabled
              ? "bg-gray-500 cursor-not-allowed opacity-70"
              : "bg-indigo-600 hover:bg-indigo-700"
          }`}
        >
          <Plus size={18} className="mr-2" />
          Add Task
        </button>
      </form>
    </div>
  );
};

export default TaskForm;