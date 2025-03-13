import React, { useState } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import { addCategory, removeCategory } from '../../redux/actions/categoryActions';
import { Trash2, Plus } from 'lucide-react';

const CategoryManager = () => {
  const [newCategory, setNewCategory] = useState('');
  const dispatch = useDispatch();
  const categories = useSelector(state => state.categories);

  const handleAddCategory = (e) => {
    e.preventDefault();
    if (!newCategory.trim() || categories.some(cat => cat.name.toLowerCase() === newCategory.toLowerCase())) {
      return;
    }
    dispatch(addCategory(newCategory.trim()));
    setNewCategory('');
  };

  const handleRemoveCategory = (id) => {
    dispatch(removeCategory(id));
  };

  return (
    <div className="bg-white dark:bg-gray-800 rounded-lg shadow-md p-5">
      <h2 className="text-lg font-semibold mb-4 text-gray-800 dark:text-gray-200">Manage Categories</h2>
      
      {/* Add Category Form */}
      <form onSubmit={handleAddCategory} className="mb-4">
        <div className="flex gap-2">
          <input
            type="text"
            value={newCategory}
            onChange={(e) => setNewCategory(e.target.value)}
            placeholder="New category name"
            className="flex-grow p-2 border border-gray-300 dark:border-gray-600 rounded-md 
                      dark:bg-gray-700 dark:text-white focus:ring-2 focus:ring-indigo-500"
          />
          <button
            type="submit"
            disabled={!newCategory.trim()}
            className={`p-2 rounded-md text-white transition duration-200 
                      ${newCategory.trim() ? 'bg-indigo-600 hover:bg-indigo-700' : 'bg-gray-400 cursor-not-allowed'}`}
          >
            <Plus size={18} />
          </button>
        </div>
      </form>
      
      {/* Category List */}
      <div className="space-y-2 max-h-60 overflow-y-auto">
        {categories.map(category => (
          <div
            key={category.id}
            className="flex items-center justify-between p-2 bg-gray-100 dark:bg-gray-700 rounded hover:bg-gray-200 dark:hover:bg-gray-600 transition"
          >
            <span className="text-gray-800 dark:text-gray-200">{category.name}</span>
            <button
              onClick={() => handleRemoveCategory(category.id)}
              className="text-red-500 hover:text-red-700 dark:text-red-400 dark:hover:text-red-300"
              disabled={category.name === 'default'} // Prevent removing default category
            >
              <Trash2 size={16} />
            </button>
          </div>
        ))}
      </div>
    </div>
  );
};

export default CategoryManager;