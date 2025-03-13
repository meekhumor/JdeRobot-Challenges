import React from 'react';
import DatePicker from 'react-datepicker';
import 'react-datepicker/dist/react-datepicker.css';
import { Calendar } from 'lucide-react';

const DueDatePicker = ({ selectedDate, onChange, disabled = false }) => {
  return (
    <div className="relative">
      {/* Calendar Icon */}
      <div className="absolute inset-y-0 left-0 pl-3 flex items-center pointer-events-none">
        <Calendar size={18} className="text-gray-400" />
      </div>
      
      {/* DatePicker Component */}
      <DatePicker
        selected={selectedDate}
        onChange={onChange}
        placeholderText="Select due date"
        dateFormat="MMMM d, yyyy"
        minDate={new Date()} // Prevent past dates
        disabled={disabled}
        className={`w-full pl-10 pr-4 py-2 border border-gray-300 dark:border-gray-600 rounded-md 
                  focus:ring-2 focus:ring-indigo-500 focus:border-indigo-500 transition
                  dark:bg-gray-700 dark:text-white dark:placeholder-gray-400
                  ${disabled ? 'bg-gray-100 cursor-not-allowed dark:bg-gray-600' : ''}`}
      />
    </div>
  );
};

export default DueDatePicker;