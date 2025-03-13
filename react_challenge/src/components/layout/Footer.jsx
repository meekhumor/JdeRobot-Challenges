import React from 'react';
import {Heart } from 'lucide-react';

const Footer = () => {
  return (
    <footer className="bg-gray-200 dark:bg-gray-800 text-gray-700 dark:text-gray-300">
      <div className="container mx-auto px-4 py-10 flex flex-col md:flex-row justify-between items-center">
        <p className="text-gray-600 dark:text-gray-400">©TaskBuddy. All rights reserved.</p>
        <p className="text-gray-600 dark:text-gray-400 flex items-center mt-4 md:mt-0">
          Made with <Heart size={16} className="text-red-500 mx-1" /> for JdeRobot GSoC 2025
        </p>
      </div>
    </footer>
  );
};

export default Footer;