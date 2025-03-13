import React from 'react';
import { useDispatch, useSelector } from 'react-redux';
import { setSearchQuery } from '../../redux/actions/filterActions';
import { Search } from 'lucide-react';

const SearchBar = () => {
  const dispatch = useDispatch();
  const searchQuery = useSelector(state => state.filter.searchQuery);

  const handleSearchChange = (e) => {
    dispatch(setSearchQuery(e.target.value));
  };

  return (
    <div className="relative">
      <div className="absolute inset-y-0 left-0 pl-3 flex items-center pointer-events-none">
        <Search size={18} className="text-gray-400" />
      </div>
      <input
        type="text"
        value={searchQuery || ''}
        onChange={handleSearchChange}
        placeholder="Search tasks..."
        className="w-full pl-10 pr-4 py-2 border border-gray-300 dark:border-gray-600 rounded-md 
                  focus:ring-2 focus:ring-indigo-500 focus:border-indigo-500 transition
                  dark:bg-gray-700 dark:text-white dark:placeholder-gray-400"
      />
    </div>
  );
};

export default SearchBar;